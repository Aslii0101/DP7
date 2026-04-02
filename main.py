from machine import Pin, PWM, ADC
import time
import network
import ujson
from umqtt.simple import MQTTClient

# ============================================================
#  CONFIG
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
#  Stops  (dead-ends / attracties)
#    0 = Depot            1 = Wildewaterbaan   2 = Arcadehal
#    3 = Reuzenrad        4 = Vrije Val Toren  5 = Achtbaan
#
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
# !! VALIDATE THESE ON THE PHYSICAL TRACK !!
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
#  BFS — shortest path in graph
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
            action = "STRAIGHT"  # fallback
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
#  STATE
# ============================================================

running = False
robot_state = "STOPPED"

# navigation
route_actions = []     # list of (node_id, "LEFT"/"RIGHT"/"STRAIGHT")
current_node = 0       # where we are (start at depot)
came_from = None       # previous node
heading_to_stop = False

# PID
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

# counters
last_direction = 1
lost_counter = 0
wide_count = 0           # consecutive frames with 3+ sensors active (intersection approach)
intersection_cooldown = 0
last_print = time.ticks_ms()

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
        except: pass

    elif topic == T_PID_KI:
        try:
            Ki = float(payload)
            I = 0.0
            print("Ki =", Ki)
        except: pass

    elif topic == T_PID_KD:
        try:
            Kd = float(payload)
            print("Kd =", Kd)
        except: pass

    elif topic == T_SPEED:
        try:
            BASE_SPEED = max(0.0, min(1.0, float(payload)))
            print("BASE_SPEED =", BASE_SPEED)
        except: pass

    elif topic == T_ROUTE:
        # full route as JSON: {"from": 0, "to": 3}  OR  ["LEFT","RIGHT","STRAIGHT"]
        try:
            data = ujson.loads(payload)

            if isinstance(data, dict) and "from" in data and "to" in data:
                # smart route: server sends start + destination, robot computes path
                start = data["from"]
                goal  = data["to"]
                path = bfs(start, goal)
                if path:
                    route_actions = path_to_actions(path)
                    current_node = start
                    came_from = None
                    heading_to_stop = goal in STOPS
                    print("Route:", start, "->", goal, "| path:", path)
                    print("Actions:", route_actions)
                    pub(T_DEBUG, "ROUTE OK: {} actions".format(len(route_actions)))
                else:
                    print("No path found:", start, "->", goal)
                    pub(T_DEBUG, "NO PATH")

            elif isinstance(data, list):
                # legacy: raw action list
                route_actions = [(None, c.upper()) for c in data
                                 if c.upper() in ("LEFT", "RIGHT", "STRAIGHT")]
                heading_to_stop = False
                print("Route set (raw):", route_actions)

        except Exception as e:
            print("Route parse error:", e)

    elif topic == T_COMMAND:
        # append single action
        cmd = payload.upper()
        if cmd in ("LEFT", "RIGHT", "STRAIGHT"):
            route_actions.append((None, cmd))
            print("Queued:", cmd, "| queue len:", len(route_actions))

# ============================================================
#  HARDWARE
# ============================================================

# motors
motor1_pwm = PWM(Pin(17), freq=60)
motor1_in1 = Pin(13, Pin.OUT)
motor2_pwm = PWM(Pin(11), freq=60)
motor2_in1 = Pin(12, Pin.OUT)

# sensors
sensor_pins = [1, 7, 6, 5, 4]
sensors = [ADC(Pin(p), atten=ADC.ATTN_11DB) for p in sensor_pins]
THRESHOLD = 30000

weights = [-2, -1, 0, 1, 2]

# ============================================================
#  MOTOR HELPERS
# ============================================================

def set_speed(left, right):
    if abs(left)  < DEADBAND: left  = 0.0
    if abs(right) < DEADBAND: right = 0.0
    motor1_in1.value(1 if right >= 0 else 0)
    motor1_pwm.duty_u16(int(65535 * abs(right)))
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


def sensor_M():
    return sensors[2].read_u16() < THRESHOLD

# ============================================================
#  PID
# ============================================================

def pid(error):
    global previousError, I, D_filtered, last_time
    now = time.ticks_ms()
    dt = max(1e-3, time.ticks_diff(now, last_time) / 1000.0)
    last_time = now

    P = error
    I += error * dt
    D_raw = (error - previousError) / dt
    D_filtered = D_alpha * D_filtered + (1 - D_alpha) * D_raw
    previousError = error

    output = Kp * P + Ki * I + Kd * D_filtered

    if output > MAX_OUTPUT:
        I -= error * dt
        output = MAX_OUTPUT
    elif output < MIN_OUTPUT:
        I -= error * dt
        output = MIN_OUTPUT

    return output


def read_error():
    vals = [s.read_u16() for s in sensors]
    inv = [65535 - v for v in vals]
    total = sum(inv)
    if total == 0:
        return None, vals
    weighted = sum(weights[i] * inv[i] for i in range(5))
    return weighted / total, vals

# ============================================================
#  TURN EXECUTION
# ============================================================

def execute_turn(action):
    """Physical turn at an intersection. Call after stopping on all-5-black."""
    if action == "STRAIGHT":
        forward()
        time.sleep(0.4)
        stop()

    elif action == "LEFT":
        forward()
        time.sleep(0.15)
        spin_left()
        time.sleep(0.15)   # leave current line
        deadline = time.ticks_ms() + 3000
        while not sensor_M() and time.ticks_diff(time.ticks_ms(), deadline) < 0:
            time.sleep(0.01)
        stop()

    elif action == "RIGHT":
        forward()
        time.sleep(0.15)
        spin_right()
        time.sleep(0.15)
        deadline = time.ticks_ms() + 3000
        while not sensor_M() and time.ticks_diff(time.ticks_ms(), deadline) < 0:
            time.sleep(0.01)
        stop()

    elif action == "UTURN":
        spin_right()
        time.sleep(0.3)
        deadline = time.ticks_ms() + 3000
        while not sensor_M() and time.ticks_diff(time.ticks_ms(), deadline) < 0:
            time.sleep(0.01)
        stop()

    time.sleep(0.05)

# ============================================================
#  MQTT PUBLISH HELPER
# ============================================================

mqtt_client = None

def pub(topic, message):
    if mqtt_client:
        try:
            mqtt_client.publish(topic, message.encode() if isinstance(message, str) else message)
        except Exception as e:
            print("MQTT pub error:", e)


def publish_status():
    pid_info = "Kp:{:.3f} Ki:{:.3f} Kd:{:.3f} err:{:.3f} I:{:.3f} D:{:.3f}".format(
        Kp, Ki, Kd, previousError, I, D_filtered)
    node_info = "node:{} from:{} queue:{}".format(current_node, came_from, len(route_actions))
    full_status = "{} | {}".format(robot_state, node_info)

    print(full_status, "|", pid_info)
    pub(T_STATUS, full_status)
    pub(T_PID_STATUS, pid_info)
    pub(T_SPEED_STATUS, "{:.2f}".format(BASE_SPEED))
    pub(T_NODE, ujson.dumps({"node": current_node, "from": came_from,
                              "queue": len(route_actions)}))

# ============================================================
#  CONNECT
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
#  MAIN LOOP
# ============================================================

print("=== LAKE SIDE MANIA ROBOT START ===")
print("PID: Kp={} Ki={} Kd={}".format(Kp, Ki, Kd))
print("Waiting for START command via MQTT...")

while True:

    # poll MQTT
    if mqtt_client:
        try:
            mqtt_client.check_msg()
        except Exception as e:
            print("MQTT check error:", e)

    # idle when stopped
    if not running:
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print) > 1000:
            publish_status()
            last_print = now
        time.sleep(0.02)
        continue

    # --- read sensors ---
    error, vals = read_error()

    L1 = vals[0] < THRESHOLD
    L2 = vals[1] < THRESHOLD
    M  = vals[2] < THRESHOLD
    R2 = vals[3] < THRESHOLD
    R1 = vals[4] < THRESHOLD

    line_seen = L1 or L2 or M or R2 or R1

    active_count = sum([L1, L2, M, R2, R1])
    if active_count >= 3:
        wide_count += 1
    else:
        wide_count = 0

    if intersection_cooldown > 0:
        intersection_cooldown -= 1

    # ── INTERSECTION: all 5 sensors black ──
    if L1 and L2 and M and R2 and R1 and intersection_cooldown == 0:

        if route_actions:
            node_id, action = route_actions.pop(0)
            robot_state = "KRUISPUNT -> {}".format(action)
            stop()
            time.sleep(0.1)

            # update navigation state
            if node_id is not None:
                came_from = current_node
                current_node = node_id
            pub(T_DEBUG, "TURN {} at node {}".format(action, current_node))

            execute_turn(action)
            intersection_cooldown = 40  # ~0.8s cooldown

            # if queue is empty and we were heading to a stop, we arrive next dead-end
            if not route_actions and heading_to_stop:
                robot_state = "ARRIVING"
                pub(T_DEBUG, "ROUTE COMPLETE")

            lost_counter = 0
            wide_count = 0

        else:
            # no route queued → drive straight through
            robot_state = "KRUISPUNT (geen route)"
            forward()
            time.sleep(0.3)
            intersection_cooldown = 40

    # ── WIDE INTERSECTION (L2+M+R2 but not all 5) ──
    elif L2 and M and R2 and not (L1 and R1) and intersection_cooldown == 0:
        if wide_count > 3:
            # slow down on approach so all-5-black can be detected cleanly
            set_speed(BASE_SPEED * 0.4, BASE_SPEED * 0.4)
            robot_state = "NADERING"
        else:
            forward()
            robot_state = "BREED RECHTDOOR"

    # ── SHARP CURVE: only L1 active ──
    elif L1 and not L2 and not M and not R2 and not R1:
        last_direction = -1
        lost_counter = 0
        wide_count = 0
        robot_state = "SCHERP LINKS"
        deadline = time.ticks_ms() + 3000
        while not sensor_M() and time.ticks_diff(time.ticks_ms(), deadline) < 0:
            spin_left()
            time.sleep(0.01)
        stop()

    # ── SHARP CURVE: only R1 active ──
    elif not L1 and not L2 and not M and not R2 and R1:
        last_direction = 1
        lost_counter = 0
        wide_count = 0
        robot_state = "SCHERP RECHTS"
        deadline = time.ticks_ms() + 3000
        while not sensor_M() and time.ticks_diff(time.ticks_ms(), deadline) < 0:
            spin_right()
            time.sleep(0.01)
        stop()

    # ── PID LINE FOLLOWING ──
    elif line_seen and error is not None:
        lost_counter = 0
        pid_out = pid(error)
        pid_out = max(-MAX_OUTPUT, min(MAX_OUTPUT, pid_out * 0.4))

        left  = max(-1.0, min(1.0, BASE_SPEED + pid_out))
        right = max(-1.0, min(1.0, BASE_SPEED - pid_out))
        set_speed(left, right)

        if error < -0.3:
            last_direction = -1
            robot_state = "LINKS ({:.2f})".format(error)
        elif error > 0.3:
            last_direction = 1
            robot_state = "RECHTS ({:.2f})".format(error)
        else:
            robot_state = "RECHT ({:.2f})".format(error)

    # ── LINE LOST: search ──
    else:
        lost_counter += 1

        if lost_counter < 20:
            if last_direction == 1:
                set_speed(0.0, SEARCH_SPEED)
                robot_state = "ZOEK BOCHT R"
            else:
                set_speed(SEARCH_SPEED, 0.0)
                robot_state = "ZOEK BOCHT L"
        elif lost_counter < 60:
            if last_direction == 1:
                spin_right()
                robot_state = "ZOEK SPIN R"
            else:
                spin_left()
                robot_state = "ZOEK SPIN L"
        elif lost_counter < 120:
            spin_right()
            robot_state = "180 ZOEK"
        else:
            # lost for too long, stop and wait
            stop()
            robot_state = "VERLOREN"
            if lost_counter == 120:
                pub(T_DEBUG, "LINE LOST - stopped")

    # ── periodic status ──
    now = time.ticks_ms()
    if time.ticks_diff(now, last_print) > 1000:
        publish_status()
        last_print = now

    time.sleep(0.02)