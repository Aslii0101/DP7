from machine import Pin, PWM, ADC
import time
import network
import ujson
from umqtt.simple import MQTTClient

# ============================================================
#  PLAN SETUP – LAKE SIDE MANIA LINE FOLLOWER
# ============================================================
# 1. The robot follows a black line on a white background.
# 2. 5 analogue sensors (leftmost to rightmost) read line position.
# 3. PID controller calculates steering correction.
# 4. Intersections (all 5 sensors black) trigger a turn based on
#    the current route (BFS shortest path) or a raw command list.
# 5. MQTT provides remote control, route planning, and PID tuning.
# 6. The graph and ROUTING tables define all possible moves.
# 7. After each turn, the robot re‑centres on the line and continues.
# ============================================================

# ============================================================
#  CONFIGURATION
# ============================================================

WIFI_SSID = "hbo-ict-iot"
WIFI_PASS = "wateenleukestudie!"
BROKER    = "mqtt.hbo-ict.net"
PORT      = 1883

# MQTT topics
T_STATUS      = b"student/AD25VBa4/robot_status"
T_CMD         = b"student/AD25VBa4/cmd"
T_PID_KP      = b"student/AD25VBa4/pid/kp"
T_PID_KI      = b"student/AD25VBa4/pid/ki"
T_PID_KD      = b"student/AD25VBa4/pid/kd"
T_SPEED       = b"student/AD25VBa4/speed"
T_PID_STATUS  = b"student/AD25VBa4/pid_status"
T_SPEED_STATUS = b"student/AD25VBa4/speed_status"
T_ROUTE       = b"student/AD25VBa4/route"
T_COMMAND     = b"student/AD25VBa4/command"
T_NODE        = b"student/AD25VBa4/node"
T_DEBUG       = b"student/AD25VBa4/debug"

# ============================================================
#  PARK GRAPH — Lake Side Mania
# ============================================================
#  Stops  (dead‑ends / attractions)
#    0 = Depot            1 = Wildewaterbaan   2 = Arcadehal
#    3 = Reuzenrad        4 = Vrije Val Toren  5 = Achtbaan
#  Intersections
#    6  = Top junction (onder depot)
#    7  = Upper-left corner
#    8  = Upper-middle junction
#    9  = Right vertical mid junction
#    10 = Lower-right curve entry
#    11 = Bottom junction (boven Achtbaan)
#    12 = Lower-left junction
#    13 = Mid-left junction

STOPS = {0, 1, 2, 3, 4, 5}

GRAPH = {
    0:  [6],
    1:  [13],
    2:  [8],
    3:  [9],
    4:  [12],
    5:  [11],
    6:  [0, 7, 9],
    7:  [6, 8, 13],
    8:  [7, 9, 2],
    9:  [6, 8, 3, 10],
    10: [9, 11],
    11: [10, 12, 5],
    12: [11, 13, 4],
    13: [12, 7, 1],
}

STOP_NAMES = {
    0: "Depot", 1: "Wildewaterbaan", 2: "Arcadehal",
    3: "Reuzenrad", 4: "VrijeValToren", 5: "Achtbaan",
}

# ROUTING[current_node][came_from][next_node] → physical action
ROUTING = {
    6:  {0: {7: "LEFT", 9: "RIGHT"},
         7: {0: "RIGHT", 9: "STRAIGHT"},
         9: {0: "LEFT", 7: "STRAIGHT"}},
    7:  {6:  {8: "STRAIGHT", 13: "LEFT"},
         8:  {6: "STRAIGHT", 13: "RIGHT"},
         13: {6: "RIGHT", 8: "LEFT"}},
    8:  {7: {9: "STRAIGHT", 2: "RIGHT"},
         9: {7: "STRAIGHT", 2: "LEFT"},
         2: {7: "LEFT", 9: "RIGHT"}},
    9:  {6:  {8: "LEFT", 3: "STRAIGHT", 10: "RIGHT"},
         8:  {6: "RIGHT", 3: "STRAIGHT", 10: "LEFT"},
         3:  {6: "LEFT", 8: "RIGHT", 10: "STRAIGHT"},
         10: {6: "STRAIGHT", 8: "RIGHT", 3: "LEFT"}},
    10: {9:  {11: "STRAIGHT"},
         11: {9: "STRAIGHT"}},
    11: {10: {12: "STRAIGHT", 5: "RIGHT"},
         12: {10: "STRAIGHT", 5: "LEFT"},
         5:  {10: "LEFT", 12: "RIGHT"}},
    12: {11: {13: "STRAIGHT", 4: "LEFT"},
         13: {11: "STRAIGHT", 4: "RIGHT"},
         4:  {11: "RIGHT", 13: "LEFT"}},
    13: {12: {7: "STRAIGHT", 1: "RIGHT"},
         7:  {12: "STRAIGHT", 1: "LEFT"},
         1:  {12: "LEFT", 7: "RIGHT"}},
}

# ============================================================
#  BFS SHORTEST PATH
# ============================================================

def bfs(start, goal):
    queue = [[start]]
    visited = {start}
    while queue:
        path = queue.pop(0)
        node = path[-1]
        if node == goal:
            return path
        for nb in GRAPH.get(node, []):
            if nb not in visited:
                visited.add(nb)
                queue.append(path + [nb])
    return None


def path_to_actions(path):
    """Convert node path to list of (node, action) tuples for each intersection."""
    actions = []
    for i in range(1, len(path) - 1):
        came_from = path[i - 1]
        current   = path[i]
        next_node = path[i + 1]
        if current in ROUTING and came_from in ROUTING[current]:
            action = ROUTING[current][came_from].get(next_node, "STRAIGHT")
        else:
            action = "STRAIGHT"
        actions.append((current, action))
    return actions

# ============================================================
#  WIFI + MQTT
# ============================================================

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASS)
    print("Connecting to WiFi...")
    for _ in range(20):
        if wlan.isconnected():
            print("WiFi connected:", wlan.ifconfig()[0])
            return True
        time.sleep(0.5)
    print("WiFi failed, running without MQTT")
    return False

# ============================================================
#  HARDWARE
# ============================================================

# motors
motor1_pwm = PWM(Pin(17), freq=60)
motor1_in1 = Pin(13, Pin.OUT)
motor2_pwm = PWM(Pin(11), freq=60)
motor2_in1 = Pin(12, Pin.OUT)

# sensors (ADC, left to right)
sensor_pins = [1, 7, 6, 5, 4]
sensors = [ADC(Pin(p), atten=ADC.ATTN_11DB) for p in sensor_pins]
THRESHOLD = 30000               # white > threshold, black < threshold
weights = [-2, -1, 0, 1, 2]     # centre = 0

# ============================================================
#  GLOBAL STATE
# ============================================================

running = False
robot_state = "STOPPED"

# navigation
route_actions = []      # list of (node_id, "LEFT"/"RIGHT"/"STRAIGHT")
current_node = 0        # start at depot
came_from = None
heading_to_stop = False

# PID variables (resetable)
Kp, Ki, Kd = 0.04, 0.0, 0.1
I = 0.0
previousError = 0.0
D_filtered = 0.0
D_alpha = 0.7
MAX_OUTPUT = 1.0
MIN_OUTPUT = -1.0
last_time = time.ticks_ms()

# speeds
BASE_SPEED   = 0.12
SHARP_SPEED  = 0.08
SEARCH_SPEED = 0.12
DEADBAND     = 0.04

# lost line & intersection handling
last_direction = 1
lost_counter = 0
wide_count = 0
intersection_cooldown = 0
last_print = time.ticks_ms()

# MQTT client
mqtt_client = None

# ============================================================
#  MOTOR HELPERS
# ============================================================

def set_speed(left, right):
    if abs(left)  < DEADBAND: left  = 0.0
    if abs(right) < DEADBAND: right = 0.0
    # right motor
    motor1_in1.value(1 if right >= 0 else 0)
    motor1_pwm.duty_u16(int(65535 * abs(right)))
    # left motor (note polarity)
    motor2_in1.value(0 if left >= 0 else 1)
    motor2_pwm.duty_u16(int(65535 * abs(left)))


def stop():
    global previousError, I, D_filtered, last_time
    set_speed(0, 0)
    previousError = 0.0
    I = 0.0
    D_filtered = 0.0
    last_time = time.ticks_ms()


def forward():
    set_speed(BASE_SPEED, BASE_SPEED)


def spin_left():
    set_speed(-SEARCH_SPEED, SEARCH_SPEED)


def spin_right():
    set_speed(SEARCH_SPEED, -SEARCH_SPEED)


def sensor_center():
    return sensors[2].read_u16() < THRESHOLD

# ============================================================
#  PID CONTROL
# ============================================================

def pid_reset():
    """Reset integral and derivative terms (called on start or Ki change)."""
    global I, previousError, D_filtered, last_time
    I = 0.0
    previousError = 0.0
    D_filtered = 0.0
    last_time = time.ticks_ms()


def pid_control(error):
    global previousError, I, D_filtered, last_time
    now = time.ticks_ms()
    dt = max(1e-3, time.ticks_diff(now, last_time) / 1000.0)
    last_time = now

    P = error
    I += error * dt
    # derivative with low‑pass filter
    D_raw = (error - previousError) / dt
    D_filtered = D_alpha * D_filtered + (1 - D_alpha) * D_raw
    previousError = error

    output = Kp * P + Ki * I + Kd * D_filtered

    # anti‑windup: clamp output and prevent integral accumulation
    if output > MAX_OUTPUT:
        I -= error * dt
        output = MAX_OUTPUT
    elif output < MIN_OUTPUT:
        I -= error * dt
        output = MIN_OUTPUT

    return output


def read_line_error():
    """Returns (normalised error, raw values). Error between -2 and +2."""
    vals = [s.read_u16() for s in sensors]
    # invert because black = low value, we want high for black
    inv = [65535 - v for v in vals]
    total = sum(inv)
    if total == 0:
        return None, vals
    weighted = sum(weights[i] * inv[i] for i in range(5))
    return weighted / total, vals

# ============================================================
#  TURN EXECUTION (improved reliability)
# ============================================================

def execute_turn(action):
    """
    Perform a physical turn at an intersection.
    Assumes robot is already stopped with all 5 sensors on black.
    """
    print("Executing turn:", action)
    pub(T_DEBUG, f"TURN {action}")

    if action == "STRAIGHT":
        # just drive forward a little to clear the intersection
        forward()
        time.sleep(0.25)
        stop()

    elif action == "LEFT":
        # step forward, then pivot left until centre sensor sees line
        forward()
        time.sleep(0.1)
        # pivot left
        start = time.ticks_ms()
        while not sensor_center() and time.ticks_diff(time.ticks_ms(), start) < 2000:
            spin_left()
            time.sleep(0.01)
        stop()
        time.sleep(0.05)

    elif action == "RIGHT":
        forward()
        time.sleep(0.1)
        start = time.ticks_ms()
        while not sensor_center() and time.ticks_diff(time.ticks_ms(), start) < 2000:
            spin_right()
            time.sleep(0.01)
        stop()
        time.sleep(0.05)

    elif action == "UTURN":
        # not used in graph, but available for raw commands
        start = time.ticks_ms()
        while not sensor_center() and time.ticks_diff(time.ticks_ms(), start) < 3000:
            spin_right()
            time.sleep(0.01)
        stop()
        time.sleep(0.05)

    # after turn, wait a moment for the line to stabilise
    time.sleep(0.1)

# ============================================================
#  MQTT CALLBACK
# ============================================================

def on_mqtt(topic, msg):
    global running, robot_state, Kp, Ki, Kd, I, BASE_SPEED
    global route_actions, current_node, came_from, heading_to_stop

    payload = msg.decode().strip()

    if topic == T_CMD:
        if payload == "START":
            running = True
            robot_state = "RUNNING"
            pid_reset()               # fresh start for PID
            print("CMD: START")
        elif payload == "STOP":
            running = False
            robot_state = "STOPPED"
            stop()
            print("CMD: STOP")

    elif topic == T_PID_KP:
        try:
            Kp = float(payload)
            print("Kp =", Kp)
        except:
            pass

    elif topic == T_PID_KI:
        try:
            Ki = float(payload)
            pid_reset()               # reset I on Ki change
            print("Ki =", Ki)
        except:
            pass

    elif topic == T_PID_KD:
        try:
            Kd = float(payload)
            print("Kd =", Kd)
        except:
            pass

    elif topic == T_SPEED:
        try:
            BASE_SPEED = max(0.0, min(1.0, float(payload)))
            print("BASE_SPEED =", BASE_SPEED)
        except:
            pass

    elif topic == T_ROUTE:
        try:
            data = ujson.loads(payload)
            if isinstance(data, dict) and "from" in data and "to" in data:
                start = data["from"]
                goal  = data["to"]
                path = bfs(start, goal)
                if path:
                    route_actions = path_to_actions(path)
                    current_node = start
                    came_from = None
                    heading_to_stop = goal in STOPS
                    print(f"Route: {start} -> {goal} | path: {path}")
                    print("Actions:", route_actions)
                    pub(T_DEBUG, f"ROUTE OK: {len(route_actions)} actions")
                else:
                    print(f"No path found: {start} -> {goal}")
                    pub(T_DEBUG, "NO PATH")
            elif isinstance(data, list):
                # raw action list (legacy or for testing)
                route_actions = [(None, c.upper()) for c in data
                                 if c.upper() in ("LEFT", "RIGHT", "STRAIGHT")]
                heading_to_stop = False
                print("Raw route set:", route_actions)
        except Exception as e:
            print("Route parse error:", e)

    elif topic == T_COMMAND:
        cmd = payload.upper()
        if cmd in ("LEFT", "RIGHT", "STRAIGHT"):
            route_actions.append((None, cmd))
            print(f"Queued: {cmd} | queue length: {len(route_actions)}")

# ============================================================
#  MQTT PUBLISH HELPER
# ============================================================

def pub(topic, message):
    if mqtt_client:
        try:
            mqtt_client.publish(topic, message.encode() if isinstance(message, str) else message)
        except Exception as e:
            print("MQTT pub error:", e)


def publish_status():
    pid_info = f"Kp:{Kp:.3f} Ki:{Ki:.3f} Kd:{Kd:.3f} err:{previousError:.3f} I:{I:.3f} D:{D_filtered:.3f}"
    node_info = f"node:{current_node} from:{came_from} queue:{len(route_actions)}"
    full_status = f"{robot_state} | {node_info}"
    print(full_status, "|", pid_info)
    pub(T_STATUS, full_status)
    pub(T_PID_STATUS, pid_info)
    pub(T_SPEED_STATUS, f"{BASE_SPEED:.2f}")
    pub(T_NODE, ujson.dumps({"node": current_node, "from": came_from,
                              "queue": len(route_actions)}))

# ============================================================
#  CONNECT TO NETWORK & MQTT
# ============================================================

if connect_wifi():
    try:
        mqtt_client = MQTTClient("dp7_robot_AD25VBa4", BROKER, PORT)
        mqtt_client.set_callback(on_mqtt)
        mqtt_client.connect()
        for t in [T_CMD, T_PID_KP, T_PID_KI, T_PID_KD, T_SPEED, T_ROUTE, T_COMMAND]:
            mqtt_client.subscribe(t)
        print("MQTT connected — subscribed to 7 topics")
    except Exception as e:
        print("MQTT connect failed:", e)
        mqtt_client = None

# ============================================================
#  MAIN CONTROL LOOP
# ============================================================

print("\n=== LAKE SIDE MANIA ROBOT (IMPROVED) ===")
print(f"PID: Kp={Kp}, Ki={Ki}, Kd={Kd} | Base speed={BASE_SPEED}")
print("Waiting for START command via MQTT...")

while True:
    # check MQTT messages
    if mqtt_client:
        try:
            mqtt_client.check_msg()
        except Exception as e:
            print("MQTT check error:", e)

    if not running:
        # idle mode – just publish status periodically
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print) > 1000:
            publish_status()
            last_print = now
        time.sleep(0.02)
        continue

    # ---------- SENSOR READING ----------
    error, raw_vals = read_line_error()

    # binary line detection (black = True)
    L1 = raw_vals[0] < THRESHOLD
    L2 = raw_vals[1] < THRESHOLD
    M  = raw_vals[2] < THRESHOLD
    R2 = raw_vals[3] < THRESHOLD
    R1 = raw_vals[4] < THRESHOLD

    active_count = sum([L1, L2, M, R2, R1])

    # handle cooldown after an intersection
    if intersection_cooldown > 0:
        intersection_cooldown -= 1

    # ---------- INTERSECTION DETECTION (all 5 black) ----------
    if L1 and L2 and M and R2 and R1 and intersection_cooldown == 0:
        # we are exactly on an intersection
        if route_actions:
            node_id, action = route_actions.pop(0)
            robot_state = f"INTERSECTION -> {action}"
            stop()
            time.sleep(0.05)

            # update navigation state if we know which node we are at
            if node_id is not None:
                came_from = current_node
                current_node = node_id
            pub(T_DEBUG, f"Turn {action} at node {current_node}")

            execute_turn(action)
            intersection_cooldown = 30          # block new intersections for ~0.6 s

            # if route finished and we were heading to a stop
            if not route_actions and heading_to_stop:
                robot_state = "ARRIVED"
                pub(T_DEBUG, "ROUTE COMPLETE")

            # reset lost counters
            lost_counter = 0
            wide_count = 0
        else:
            # no route – drive straight through (should not happen in normal operation)
            robot_state = "INTERSECTION (free)"
            forward()
            time.sleep(0.3)
            intersection_cooldown = 30

    # ---------- WIDE INTERSECTION (L2+M+R2 but not all 5) ----------
    elif L2 and M and R2 and not (L1 and R1) and intersection_cooldown == 0:
        # slow down to reliably detect the full intersection
        set_speed(BASE_SPEED * 0.4, BASE_SPEED * 0.4)
        robot_state = "APPROACHING"
        wide_count += 1
        # after a few frames, the full intersection will trigger

    # ---------- SHARP LEFT CURVE (only leftmost sensor) ----------
    elif L1 and not L2 and not M and not R2 and not R1:
        last_direction = -1
        lost_counter = 0
        wide_count = 0
        robot_state = "SHARP LEFT"
        start = time.ticks_ms()
        while not sensor_center() and time.ticks_diff(time.ticks_ms(), start) < 2000:
            spin_left()
            time.sleep(0.01)
        stop()

    # ---------- SHARP RIGHT CURVE (only rightmost sensor) ----------
    elif not L1 and not L2 and not M and not R2 and R1:
        last_direction = 1
        lost_counter = 0
        wide_count = 0
        robot_state = "SHARP RIGHT"
        start = time.ticks_ms()
        while not sensor_center() and time.ticks_diff(time.ticks_ms(), start) < 2000:
            spin_right()
            time.sleep(0.01)
        stop()

    # ---------- NORMAL LINE FOLLOWING (PID) ----------
    elif (L1 or L2 or M or R2 or R1) and error is not None:
        lost_counter = 0
        wide_count = 0
        pid_out = pid_control(error)
        # scale steering a bit for smoother behaviour
        pid_out = max(-MAX_OUTPUT, min(MAX_OUTPUT, pid_out * 0.4))

        left = BASE_SPEED + pid_out
        right = BASE_SPEED - pid_out
        set_speed(left, right)

        # remember last turn direction for line recovery
        if error < -0.3:
            last_direction = -1
            robot_state = f"LEFT ({error:.2f})"
        elif error > 0.3:
            last_direction = 1
            robot_state = f"RIGHT ({error:.2f})"
        else:
            robot_state = f"STRAIGHT ({error:.2f})"

    # ---------- LINE LOST – SEARCH PROCEDURE ----------
    else:
        lost_counter += 1
        if lost_counter < 20:
            # gentle wiggle
            if last_direction == 1:
                set_speed(0.0, SEARCH_SPEED)
                robot_state = "SEARCH R"
            else:
                set_speed(SEARCH_SPEED, 0.0)
                robot_state = "SEARCH L"
        elif lost_counter < 60:
            # pivot
            if last_direction == 1:
                spin_right()
                robot_state = "PIVOT R"
            else:
                spin_left()
                robot_state = "PIVOT L"
        elif lost_counter < 120:
            # desperate wide spin
            spin_right()
            robot_state = "DESPERATE SPIN"
        else:
            # completely lost – stop and wait for human
            stop()
            robot_state = "LOST"
            if lost_counter == 120:
                pub(T_DEBUG, "LINE LOST - stopped")

    # ---------- PERIODIC STATUS ----------
    now = time.ticks_ms()
    if time.ticks_diff(now, last_print) > 1000:
        publish_status()
        last_print = now

    time.sleep(0.02)
