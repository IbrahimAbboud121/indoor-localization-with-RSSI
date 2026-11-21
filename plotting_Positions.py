import socket, math, time
import matplotlib.pyplot as plt

ROOM_W, ROOM_H = 2.6, 4.3
ANCHORS = [(0.2, 0.2), (2.4, 0.2), (1.3, 4.1)]     # EDIT: your measured anchor coordinates
POSITIONS = [(0.40, 0.60), (2.10, 0.60), (2.10, 3.30),(0.40, 3.30) , (1.25, 1.80)]
HEAD0_BEARING = 180.0 
HEAD_SIGN = 1.0 
ANIM_SEC = 1.2          # duration of the glide between positions
NAMES = ["MOVING", "MEASURING...", "AT POSITION"]

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", 5005)); sock.settimeout(0.01)

plt.ion()
fig, ax = plt.subplots(figsize=(5, 8))
ax.set_xlim(-0.2, ROOM_W + 0.2); ax.set_ylim(-0.2, ROOM_H + 0.2); ax.set_aspect("equal")
ax.plot([0, ROOM_W, ROOM_W, 0, 0], [0, 0, ROOM_H, ROOM_H, 0], "k-")
for i, (x, y) in enumerate(ANCHORS):
    ax.plot(x, y, "b^", ms=10); ax.text(x + 0.05, y + 0.05, f"A{i+1}", color="b")
for i, (x, y) in enumerate(POSITIONS):
    ax.plot(x, y, "o", mfc="none", mec="gray", ms=14); ax.text(x + 0.08, y - 0.12, f"P{i+1}", color="gray")

seg,  = ax.plot([], [], "-", color="tomato", lw=2.5)             # latest segment only
cur,  = ax.plot([], [], "o", color="red", ms=15, mec="k", zorder=5)
arrow = None

disp = None                       # displayed position (x, y)
a_from = a_to = None
a_t0 = 0.0
heading, state, n_pkts, n_T = 0.0, 2, 0, 0

def start_glide(target):
    global a_from, a_to, a_t0
    a_from, a_to, a_t0 = disp, target, time.time()

def update_motion():
    global disp
    if a_to is None or disp is None:
        return
    f = min(1.0, (time.time() - a_t0) / ANIM_SEC)
    e = f * f * (3 - 2 * f)                                      # smoothstep ease in/out
    disp = (a_from[0] + (a_to[0] - a_from[0]) * e, a_from[1] + (a_to[1] - a_from[1]) * e)
    seg.set_data([a_from[0], disp[0]], [a_from[1], disp[1]])
    if f >= 1.0:
        seg.set_data([], [])                                     # glide finished: erase the line

def redraw():
    global arrow
    if disp is not None:
        cur.set_data([disp[0]], [disp[1]])
        ang = math.radians(HEAD0_BEARING + HEAD_SIGN * heading)
        if arrow: arrow.remove()
        arrow = ax.arrow(disp[0], disp[1], 0.4 * math.sin(ang), 0.4 * math.cos(ang),
                         head_width=0.08, color="g", zorder=6)
    st = NAMES[state] if n_T else "press the button on the tag"
    ax.set_title(f"pkts={n_pkts}  T={n_T}  {st}  hdg={heading:.0f}", fontsize=9)

while plt.fignum_exists(fig.number):
    try:
        data = sock.recvfrom(1024)[0].decode(errors="ignore")
        n_pkts += 1
    except socket.timeout:
        data = ""
    for raw in data.splitlines():
        raw = raw.strip(); f = raw.split(",")
        try:
            if raw.startswith("#"):
                print(raw)
            elif f[0] == "S":                                    # test started: reset everything
                print(raw); disp = POSITIONS[0]; a_to = None; seg.set_data([], [])
            elif f[0] == "T":
                n_T += 1; state, heading = int(f[2]), float(f[6])
                if disp is None:
                    disp = (float(f[4]), float(f[5]))
            elif f[0] == "P":                                    # confirmed move
                print(raw)
                if disp is None: disp = (float(f[3]), float(f[4]))
                else: start_glide((float(f[3]), float(f[4])))
        except (ValueError, IndexError) as e:
            print("parse error:", raw, e)
    update_motion()
    redraw()
    plt.pause(0.02)