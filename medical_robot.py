import browserbotics as bb
import time
import math

# ═══════════════════════════════════════════════════════════════════════════════
#  SURGICAL ROBOT v3 — AUTO + MANUAL MODE
#
#  MODE slider:  0 = AUTO,   1 = MANUAL
#
#  AUTO:   AUTO_START=1 runs full sequence automatically
#          AUTO_START=0 resets to corner
#
#  MANUAL: MANUAL_X / MANUAL_Y  — drag to drive cart smoothly anywhere
#          MANUAL_SPEED          — how fast cart drives to target
#          J0 … J6               — live joint control, updates every frame
# ═══════════════════════════════════════════════════════════════════════════════

ROBOT_BASE_Z = 0.68
RESPAWN_THRESHOLD = 0.04   # only respawn robot when cart moves this far

POS_CORNER  = [4.50, -4.50]
POS_TRAY    = [1.40, -0.50]
POS_BEDSIDE = [0.75, -0.50]

ROUTE_TO_TRAY    = [[4.50,-4.50],[4.50,-0.50],[1.40,-0.50]]
ROUTE_TO_BEDSIDE = [[1.40,-0.50],[0.75,-0.50]]
ROUTE_RETURN     = [[0.75,-0.50],[1.40,-0.50],[4.50,-0.50],[4.50,-4.50]]

J0_TRAY    =  1.57
J0_PATIENT = -0.98

POSE_CARRY   = [J0_PATIENT, -0.3,  0.0, -1.8,  0.0,  1.6,  0.7]
POSE_REACH   = [J0_TRAY,     0.5,  0.0, -1.0,  0.0,  1.5,  0.7]
POSE_GRIP    = [J0_TRAY,     0.7,  0.0, -0.8,  0.0,  1.4,  0.7]
POSE_HOVER   = [J0_PATIENT,  0.2,  0.0, -1.6,  0.0,  1.9,  0.4]
POSE_OPERATE = [J0_PATIENT,  0.5,  0.1, -1.1,  0.0,  1.6,  0.3]

def lerp(a, b, t):
    t = max(0.0, min(1.0, t))
    return [a[i]+(b[i]-a[i])*t for i in range(min(len(a),len(b)))]


# ═══════════════════════════════════════════════════════════════════════════════
#  ROOM
# ═══════════════════════════════════════════════════════════════════════════════
def build_room():
    bb.addGroundPlane()
    W, H = 6, 0.15
    bb.createBody("box", halfExtent=[0.05,W,H], position=[-W,0,H], color="#B2DFDB", mass=0)
    bb.createBody("box", halfExtent=[0.05,W,H], position=[ W,0,H], color="#B2DFDB", mass=0)
    bb.createBody("box", halfExtent=[W,0.05,H], position=[0,-W,H], color="#B2DFDB", mass=0)
    bb.createBody("box", halfExtent=[W,0.05,H], position=[0, W,H], color="#B2DFDB", mass=0)
    for lx,ly in [(-0.4,-0.9),(0.4,-0.9),(-0.4,0.9),(0.4,0.9)]:
        bb.createBody("box", halfExtent=[0.04,0.04,0.3], position=[lx,ly,0.3], color="#9E9E9E", mass=0)
    bb.createBody("box", halfExtent=[0.5,1.05,0.04],   position=[0,0,0.64],    color="#CFD8DC", mass=0)
    bb.createBody("box", halfExtent=[0.45,0.85,0.015], position=[0,0,0.695],   color="#388E3C", mass=0)
    bb.createBody("box", halfExtent=[0.28,0.16,0.03],  position=[0,0.82,0.74], color="#FFFDE7", mass=0)
    bb.createBody("box",    halfExtent=[0.18,0.40,0.10], position=[0, 0.20,0.81],     color="#388E3C", mass=0)
    bb.createBody("box",    halfExtent=[0.09,0.35,0.08], position=[-0.14,-0.50,0.79], color="#388E3C", mass=0)
    bb.createBody("box",    halfExtent=[0.09,0.35,0.08], position=[ 0.14,-0.50,0.79], color="#388E3C", mass=0)
    bb.createBody("sphere", radius=0.12, position=[0,0.85,0.89], color="#FDBCB4", mass=0)
    bb.createBody("box", halfExtent=[0.04,0.5,0.01], position=[-0.57,0,0.69], color="#ECEFF1", mass=0)
    bb.createBody("box", halfExtent=[0.04,0.5,0.01], position=[ 0.57,0,0.69], color="#ECEFF1", mass=0)
    bb.createBody("box",    halfExtent=[0.03,0.03,0.4], position=[0,0,2.6],  color="#EEEEEE", mass=0)
    bb.createBody("box",    halfExtent=[0.35,0.03,0.03],position=[0,0,2.2],  color="#EEEEEE", mass=0)
    bb.createBody("sphere", radius=0.22, position=[0,0,2.1],  color="#FFFF99", mass=0)
    bb.createBody("sphere", radius=0.17, position=[0,0,1.88], color="#FFFFFF", mass=0)
    bb.createBody("box", halfExtent=[0.02,0.02,0.48],  position=[POS_TRAY[0],POS_TRAY[1],0.48],  color="#9E9E9E", mass=0)
    bb.createBody("box", halfExtent=[0.28,0.20,0.012], position=[POS_TRAY[0],POS_TRAY[1],0.98],  color="#ECEFF1", mass=0)
    bb.createBody("box", halfExtent=[0.012,0.09,0.008],position=[POS_TRAY[0]-0.10,POS_TRAY[1]-0.04,1.00],color="#EF9A9A",mass=0)
    bb.createBody("box", halfExtent=[0.012,0.11,0.008],position=[POS_TRAY[0]-0.02,POS_TRAY[1]+0.00,1.00],color="#A5D6A7",mass=0)
    bb.createBody("box", halfExtent=[0.012,0.07,0.008],position=[POS_TRAY[0]+0.08,POS_TRAY[1]+0.04,1.00],color="#90CAF9",mass=0)
    bb.createBody("box", halfExtent=[0.012,0.08,0.008],position=[POS_TRAY[0]+0.16,POS_TRAY[1]-0.04,1.00],color="#FFE082",mass=0)
    bb.createBody("box", halfExtent=[0.28,0.22,0.55], position=[-1.15,1.45,0.55], color="#546E7A", mass=0)
    bb.createBody("box", halfExtent=[0.18,0.03,0.14], position=[-1.15,1.25,1.24], color="#1A237E", mass=0)
    bb.createBody("box", halfExtent=[0.14,0.01,0.10], position=[-1.15,1.24,1.24], color="#00E5FF", mass=0)
    bb.createBody("box", halfExtent=[0.04,0.04,0.30], position=[-0.78,1.52,0.30], color="#42A5F5", mass=0)
    bb.createBody("box", halfExtent=[0.04,0.04,0.30], position=[-0.86,1.52,0.30], color="#66BB6A", mass=0)
    bb.createBody("box", halfExtent=[0.04,0.04,0.30], position=[-0.94,1.52,0.30], color="#FFA726", mass=0)
    bb.createBody("box", halfExtent=[0.02,0.02,0.65], position=[-1.95,-0.65,0.65], color="#555555", mass=0)
    bb.createBody("box", halfExtent=[0.22,0.06,0.16], position=[-1.95,-0.65,1.46], color="#263238", mass=0)
    bb.createBody("box", halfExtent=[0.18,0.02,0.12], position=[-1.95,-0.60,1.46], color="#00E676", mass=0)
    sx,sy = -0.90,-0.25
    bb.createBody("box",    halfExtent=[0.06,0.10,0.22], position=[sx-0.07,sy,0.22], color="#00897B", mass=0)
    bb.createBody("box",    halfExtent=[0.06,0.10,0.22], position=[sx+0.07,sy,0.22], color="#00897B", mass=0)
    bb.createBody("box",    halfExtent=[0.14,0.12,0.22], position=[sx,sy,0.66],      color="#00897B", mass=0)
    bb.createBody("box",    halfExtent=[0.04,0.10,0.04], position=[sx-0.19,sy,0.74], color="#FDBCB4", mass=0)
    bb.createBody("box",    halfExtent=[0.04,0.10,0.04], position=[sx+0.19,sy,0.74], color="#FDBCB4", mass=0)
    bb.createBody("sphere", radius=0.11, position=[sx,sy,0.99],  color="#FDBCB4", mass=0)
    bb.createBody("box",    halfExtent=[0.10,0.10,0.04], position=[sx,sy,1.14],     color="#26C6DA", mass=0)
    bb.createBody("box",    halfExtent=[0.06,0.10,0.22], position=[-0.07,1.72,0.22],color="#1565C0", mass=0)
    bb.createBody("box",    halfExtent=[0.06,0.10,0.22], position=[ 0.07,1.72,0.22],color="#1565C0", mass=0)
    bb.createBody("box",    halfExtent=[0.14,0.12,0.22], position=[0.0,1.72,0.66],  color="#1565C0", mass=0)
    bb.createBody("sphere", radius=0.11, position=[0.0,1.72,0.99], color="#FDBCB4", mass=0)
    bb.createBody("box", halfExtent=[0.015,0.015,1.0], position=[0.75,1.35,1.0],  color="#CCCCCC", mass=0)
    bb.createBody("box", halfExtent=[0.25, 0.015,0.01],position=[0.75,1.35,2.0],  color="#CCCCCC", mass=0)
    bb.createBody("box", halfExtent=[0.07, 0.025,0.12],position=[0.60,1.35,1.85], color="#80D8FF", mass=0)
    bb.createBody("box", halfExtent=[0.07, 0.025,0.10],position=[0.90,1.35,1.82], color="#FFCC80", mass=0)
    bb.createBody("box", halfExtent=[0.28,0.22,0.55], position=[3.5,-2.0,0.55],  color="#D32F2F", mass=0)
    for di in range(3):
        bb.createBody("box", halfExtent=[0.25,0.21,0.06],
                      position=[3.5,-1.80,0.12+di*0.34], color="#EF9A9A", mass=0)
    bb.createBody("box", halfExtent=[0.10,0.10,0.15], position=[-2.00,-2.5,0.15], color="#FFEE58", mass=0)
    bb.createBody("box", halfExtent=[0.08,0.08,0.12], position=[-2.25,-2.5,0.12], color="#EF5350", mass=0)
    bb.createBody("box", halfExtent=[0.10,0.10,0.15], position=[-1.75,-2.5,0.15], color="#424242", mass=0)
    for cx,cy in [(-3,-3),(3,-3),(-3,3),(3,3)]:
        bb.createBody("box", halfExtent=[0.40,0.40,0.04], position=[cx,cy,2.96], color="#FFFDE7", mass=0)
        bb.createBody("box", halfExtent=[0.35,0.35,0.01], position=[cx,cy,2.91], color="#FFF9C4", mass=0)


# ═══════════════════════════════════════════════════════════════════════════════
#  CART — tracked IDs, rebuilt only when position changes enough
# ═══════════════════════════════════════════════════════════════════════════════
cart_ids = []

def build_cart(rx, ry):
    global cart_ids
    ids = []
    for wx,wy in [(-0.22,-0.22),(0.22,-0.22),(-0.22,0.22),(0.22,0.22)]:
        ids.append(bb.createBody("sphere", radius=0.08,
                                 position=[rx+wx,ry+wy,0.08], color="#111111", mass=0))
    ids.append(bb.createBody("box", halfExtent=[0.22,0.02,0.02],
                              position=[rx,ry-0.22,0.08], color="#546E7A", mass=0))
    ids.append(bb.createBody("box", halfExtent=[0.22,0.02,0.02],
                              position=[rx,ry+0.22,0.08], color="#546E7A", mass=0))
    ids.append(bb.createBody("box", halfExtent=[0.30,0.30,0.06],
                              position=[rx,ry,0.22], color="#455A64", mass=0))
    ids.append(bb.createBody("box", halfExtent=[0.18,0.015,0.015],
                              position=[rx,ry-0.32,0.25], color="#FF6F00", mass=0))
    ids.append(bb.createBody("box", halfExtent=[0.18,0.015,0.015],
                              position=[rx,ry+0.32,0.25], color="#FF6F00", mass=0))
    ids.append(bb.createBody("box", halfExtent=[0.22,0.22,0.18],
                              position=[rx,ry,0.46], color="#37474F", mass=0))
    ids.append(bb.createBody("box", halfExtent=[0.24,0.24,0.02],
                              position=[rx,ry,0.66], color="#263238", mass=0))
    cart_ids = ids

def remove_cart():
    global cart_ids
    for bid in cart_ids:
        try: bb.removeBody(bid)
        except: pass
    cart_ids = []


# ═══════════════════════════════════════════════════════════════════════════════
#  ROBOT
# ═══════════════════════════════════════════════════════════════════════════════
def spawn_robot(rx, ry):
    r = bb.loadURDF("panda.urdf", [rx, ry, ROBOT_BASE_Z], fixedBase=True)
    jts = []
    for i in range(bb.getNumJoints(r)):
        jname, jtype, jlimits = bb.getJointInfo(r, i)
        if jtype != "fixed":
            jts.append((i, jname, jlimits))
    return r, jts

def set_arm(robot, jts, pose):
    for idx,(ji,jname,jlim) in enumerate(jts):
        if idx >= len(pose): break
        bb.setJointMotorControl(robot, ji,
            targetPosition=max(jlim[0], min(jlim[1], pose[idx])))

def full_respawn(rx, ry, pose, robot, jts):
    """Remove robot + cart, rebuild both at new position."""
    bb.removeBody(robot)
    remove_cart()
    build_cart(rx, ry)
    robot, jts = spawn_robot(rx, ry)
    set_arm(robot, jts, pose)
    return robot, jts

def drive_step(rx, ry, tx, ty, spd):
    dx,dy = tx-rx, ty-ry
    dist  = math.sqrt(dx*dx+dy*dy)
    if dist < 0.05: return tx, ty, True
    s = min(spd, dist)
    return rx+dx/dist*s, ry+dy/dist*s, False


# ═══════════════════════════════════════════════════════════════════════════════
#  INIT
# ═══════════════════════════════════════════════════════════════════════════════
build_room()
rx, ry = POS_CORNER
build_cart(rx, ry)
robot, jts = spawn_robot(rx, ry)
set_arm(robot, jts, POSE_CARRY)

# ── SLIDERS ───────────────────────────────────────────────────────────────────
bb.addDebugSlider("MODE_0auto_1manual",  0,    0,    1   )

# Auto sliders
bb.addDebugSlider("AUTO_START",          0,    0,    1   )
bb.addDebugSlider("AUTO_SPEED",          0.06, 0.02, 0.15)

# Manual cart — drag X/Y live, cart smoothly follows
bb.addDebugSlider("MANUAL_X",    rx,   -5.0, 5.0)
bb.addDebugSlider("MANUAL_Y",    ry,   -5.0, 5.0)
bb.addDebugSlider("MANUAL_SPEED",0.08, 0.01, 0.20)

# Manual joint sliders — named clearly J0..J6 with degree hints
joint_sliders = []   # list of (slider_name, ji, jlim)
for idx,(ji,jname,jlim) in enumerate(jts):
    lo, hi = jlim
    default = POSE_CARRY[idx] if idx < len(POSE_CARRY) else (lo+hi)/2
    sname = f"J{idx}  ({math.degrees(lo):.0f} to {math.degrees(hi):.0f} deg)"
    bb.addDebugSlider(sname, default, lo, hi)
    joint_sliders.append((sname, ji, jlim))

print("\n========================================")
print("  SURGICAL ROBOT  —  v3")
print("========================================")
print("MODE=0  AUTO:   slide AUTO_START to 1")
print("MODE=1  MANUAL: drag MANUAL_X / MANUAL_Y to drive cart")
print("                drag J0..J6 to pose arm live")
print("========================================")

# ── State ─────────────────────────────────────────────────────────────────────
stage=0; wp_idx=0; arm_t=0.0; hold_ticks=0
prev_start=0; prev_mode=-1; cur_route=ROUTE_TO_TRAY
last_spawn_x, last_spawn_y = rx, ry   # track where robot was last spawned

LABELS = {
    0:"IDLE — AUTO_START=1 to begin",
    1:"Driving to tray...",
    2:"Picking up tool...",
    3:"Driving to bedside...",
    4:"Surgery...",
    5:"Retracting...",
    6:"Returning to corner...",
    7:"DONE — reset AUTO_START to 0",
}
def say(s): print(f"\n>>> STAGE {s}: {LABELS[s]}")
say(0)


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN LOOP
# ═══════════════════════════════════════════════════════════════════════════════
while True:
    mode      = bb.readDebugParameter("MODE_0auto_1manual")
    is_manual = mode > 0.5

    # ── MODE SWITCH ───────────────────────────────────────────────────────────
    if is_manual and prev_mode != 1:
        stage=arm_t=hold_ticks=wp_idx=0; prev_start=0
        # Sync manual sliders to current cart position
        bb.addDebugSlider("MANUAL_X", rx, -5.0, 5.0)
        bb.addDebugSlider("MANUAL_Y", ry, -5.0, 5.0)
        print("\n>>> MANUAL MODE")
        print("    Drag MANUAL_X / MANUAL_Y  — cart drives smoothly to target")
        print("    Drag J0..J6               — arm joints update live")
        prev_mode = 1

    elif not is_manual and prev_mode != 0:
        stage=arm_t=hold_ticks=wp_idx=0; prev_start=0
        print("\n>>> AUTO MODE — slide AUTO_START to 1")
        prev_mode = 0

    # ══════════════════════════════════════════════════════════════════════════
    #  MANUAL MODE
    # ══════════════════════════════════════════════════════════════════════════
    if is_manual:
        mspeed = bb.readDebugParameter("MANUAL_SPEED")
        tx     = bb.readDebugParameter("MANUAL_X")
        ty     = bb.readDebugParameter("MANUAL_Y")

        # Read current joint poses from sliders
        manual_pose = [bb.readDebugParameter(sn) for sn,ji,jlim in joint_sliders]

        # Smoothly drive cart toward target X/Y one step per frame
        dx, dy = tx-rx, ty-ry
        dist   = math.sqrt(dx*dx + dy*dy)

        if dist > RESPAWN_THRESHOLD:
            # Move one step toward target
            step = min(mspeed, dist)
            rx  += dx/dist * step
            ry  += dy/dist * step

            # Only do full respawn when moved far enough (avoid constant flicker)
            moved = math.sqrt((rx-last_spawn_x)**2 + (ry-last_spawn_y)**2)
            if moved >= RESPAWN_THRESHOLD:
                robot, jts = full_respawn(rx, ry, manual_pose, robot, jts)
                # Rebuild joint_sliders list with new jts
                joint_sliders = []
                for idx,(ji,jname,jlim) in enumerate(jts):
                    lo,hi = jlim
                    sname = f"J{idx}  ({math.degrees(lo):.0f} to {math.degrees(hi):.0f} deg)"
                    joint_sliders.append((sname, ji, jlim))
                last_spawn_x, last_spawn_y = rx, ry
        else:
            # Cart is at target — just update arm joints live every frame
            set_arm(robot, jts, manual_pose)

    # ══════════════════════════════════════════════════════════════════════════
    #  AUTO MODE
    # ══════════════════════════════════════════════════════════════════════════
    else:
        speed     = bb.readDebugParameter("AUTO_SPEED")
        start_now = bb.readDebugParameter("AUTO_START")

        # Reset
        if start_now < 0.5 and stage not in (0,7):
            robot,jts = full_respawn(POS_CORNER[0],POS_CORNER[1],POSE_CARRY,robot,jts)
            rx,ry = POS_CORNER
            last_spawn_x,last_spawn_y = rx,ry
            stage=arm_t=hold_ticks=wp_idx=0; prev_start=0
            say(0)

        # Start
        if start_now > 0.5 and prev_start < 0.5 and stage == 0:
            stage=1; wp_idx=1; cur_route=ROUTE_TO_TRAY; arm_t=0.0
            say(1)
        prev_start = start_now

        # Stage 1: Drive to tray
        if stage == 1:
            tx,ty = cur_route[wp_idx]
            rx,ry,arrived = drive_step(rx,ry,tx,ty,speed)
            robot,jts = full_respawn(rx,ry,POSE_CARRY,robot,jts)
            last_spawn_x,last_spawn_y = rx,ry
            if arrived:
                if abs(tx-POS_TRAY[0])<0.1 and abs(ty-POS_TRAY[1])<0.1:
                    stage=2; arm_t=0.0; say(2)
                else: wp_idx+=1

        # Stage 2: Pick up tool
        elif stage == 2:
            arm_t = min(1.0, arm_t+0.008)
            if   arm_t < 0.30: pose = lerp(POSE_CARRY, POSE_REACH, arm_t/0.30)
            elif arm_t < 0.55: pose = lerp(POSE_REACH,  POSE_GRIP,  (arm_t-0.30)/0.25)
            elif arm_t < 0.70: pose = lerp(POSE_GRIP,   POSE_REACH, (arm_t-0.55)/0.15)
            else:              pose = lerp(POSE_REACH,  POSE_CARRY, (arm_t-0.70)/0.30)
            set_arm(robot,jts,pose)
            if arm_t >= 1.0:
                stage=3; wp_idx=1; cur_route=ROUTE_TO_BEDSIDE; arm_t=0.0; say(3)

        # Stage 3: Drive to bedside
        elif stage == 3:
            tx,ty = cur_route[wp_idx]
            rx,ry,arrived = drive_step(rx,ry,tx,ty,speed*0.4)
            robot,jts = full_respawn(rx,ry,POSE_CARRY,robot,jts)
            last_spawn_x,last_spawn_y = rx,ry
            if arrived:
                if abs(tx-POS_BEDSIDE[0])<0.1 and abs(ty-POS_BEDSIDE[1])<0.1:
                    stage=4; arm_t=0.0; hold_ticks=220; say(4)
                else: wp_idx+=1

        # Stage 4: Surgery
        elif stage == 4:
            arm_t = min(1.0, arm_t+0.008)
            hold_ticks = max(0,hold_ticks-1)
            if   arm_t < 0.35: pose = lerp(POSE_CARRY,   POSE_HOVER,   arm_t/0.35)
            elif arm_t < 0.65: pose = lerp(POSE_HOVER,   POSE_OPERATE, (arm_t-0.35)/0.30)
            else:
                osc  = 0.03*math.sin(arm_t*math.pi*18)
                pose = [POSE_OPERATE[j]+(osc if j in [1,3] else 0)
                        for j in range(len(POSE_OPERATE))]
            set_arm(robot,jts,pose)
            if hold_ticks == 0: stage=5; arm_t=0.0; say(5)

        # Stage 5: Retract
        elif stage == 5:
            arm_t = min(1.0, arm_t+0.010)
            if   arm_t < 0.40: pose = lerp(POSE_OPERATE, POSE_HOVER,  arm_t/0.40)
            else:               pose = lerp(POSE_HOVER,   POSE_CARRY,  (arm_t-0.40)/0.60)
            set_arm(robot,jts,pose)
            if arm_t >= 1.0: stage=6; wp_idx=1; cur_route=ROUTE_RETURN; say(6)

        # Stage 6: Return
        elif stage == 6:
            tx,ty = cur_route[wp_idx]
            rx,ry,arrived = drive_step(rx,ry,tx,ty,speed)
            robot,jts = full_respawn(rx,ry,POSE_CARRY,robot,jts)
            last_spawn_x,last_spawn_y = rx,ry
            if arrived:
                if abs(tx-POS_CORNER[0])<0.1 and abs(ty-POS_CORNER[1])<0.1:
                    stage=7; say(7)
                else: wp_idx+=1

        elif stage == 7:
            pass

    time.sleep(0.05)
