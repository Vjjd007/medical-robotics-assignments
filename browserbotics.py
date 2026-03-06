import browserbotics as bb
import time
import math

# =============================================================================
# TWO-ROOM MEDICAL ROBOT — v4
#
# CHANGES FROM v3:
#   ── BUG FIXES ──
#   • Robot no longer phases through shared wall — door waypoints pushed further
#     apart so robot body clears wall thickness at all times
#   • Robot no longer clips dispensing counter — approach waypoint moved back
#   • Robot no longer clips bedside table — approach waypoint adjusted
#   • Gripper pick height raised so fingers don't clip counter surface
#   • IV stand repositioned slightly so it is off the robot's approach path
#   • Room signs raised well above walls so they are always readable
#   • Title HUD lines spaced further apart, no overlap
#   • Status HUD moved above scene, never occluded by robot
#   • turn_to steps now scale with angle so no jarring fast spin
#   • Inter-run reset has a visible pause + status message before re-running
#   ── REMOVED ──
#   • random.choice removed — colors are deterministic now
#   • face_yaw() dead code removed
#   • _hud global list replaced with clean global variable
#   ── CONTROLS ──
#   • WASD / arrow keys  — drive robot (manual mode)
#   • I / K              — arm shoulder pitch up / down
#   • J / L              — arm yaw left / right
#   • U / O              — elbow flex / extend
#   • G                  — toggle gripper open / closed
#   • D key (delivery)   — trigger one delivery run
#   • R key (reset)      — move tablet back to counter, robot back to dock
#   • SPACE              — toggle between manual and auto mode
#
# LAYOUT  (bird's-eye):
#
#   Y+  ╔══════════════════╦═══╤═══╦══════════════════╗
#       ║  PHARMACY ROOM   ║   │   ║   PATIENT ROOM   ║
#       ║                  ║   │   ║                  ║
#  Y=0  ║  [ROBOT DOCK]    ║ ◄─┤─► ║   [BED+TABLE]   ║
#       ║                  ║   │   ║                  ║
#       ║  (OPEN FRONT)    ║   │   ║   (OPEN FRONT)   ║
#   Y-  ╚══════════════════╩═══╧═══╩══════════════════╝
#       X: -14            -6   0  +6                 +8
# =============================================================================

TICK = 0.020

# ── Room geometry ─────────────────────────────────────────────────────────────
PH_X    = -10.0
PT_X    =   2.0
ROOM_HW =   4.0
ROOM_HD =   3.8
H       =   3.0

WALL_X  = PH_X + ROOM_HW   # = -6.0
WALL_T  = 0.12              # wall half-thickness

DOOR_W  = 1.5               # half-width of door gap

# Key positions
TABLET_X,  TABLET_Y,  TABLET_Z  = -8.2,  0.4,  1.08
BEDSIDE_X, BEDSIDE_Y, BEDSIDE_Z =  0.6, -0.8,  1.08
DOCK_X,    DOCK_Y               = -12.5,  0.0

# ── Door crossing waypoints ────────────────────────────────────────────────────
# Robot base X half-extent = 0.34
# Wall spans X: WALL_X - WALL_T  to  WALL_X + WALL_T  =  -6.12 to -5.88
# We need robot centre to be at least 0.34 + WALL_T = 0.46 away from wall centre
# so safe clearance = 0.55 each side
DOOR_EXIT_X  = WALL_X - 0.55   # = -6.55  (pharmacy side, body fully clear of wall)
DOOR_ENTRY_X = WALL_X + 0.55   # = -5.45  (patient side, body fully clear of wall)

# ── Counter approach ──────────────────────────────────────────────────────────
# Counter occupies Y: 0.30-0.50 = -0.20 to +0.80
# Robot base Y half-extent = 0.40  →  safe approach Y = -0.20 - 0.40 - 0.15 = -0.75
# Use Y offset of -1.0 from TABLET_Y to be safe
COUNTER_APPROACH_Y_OFFSET = -1.0

# ── Bedside approach ──────────────────────────────────────────────────────────
# IV stand at bx+0.88, 1.68 — robot approaches from Y-
# Use Y offset of -1.1 to stay clear of table arm and IV stand
BEDSIDE_APPROACH_Y_OFFSET = -1.1

Q0 = None


# ─────────────────────────────────────────────────────────────────────────────
# MATH HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def ease(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)

def lerp(a, b, t):
    return a + (b - a) * t

def norm_angle(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def place(bid, x, y, z, rx=0, ry=0, rz=None):
    if rz is None: rz = 0
    bb.resetBasePose(bid, [x, y, z],
                     bb.getQuaternionFromEuler([rx, ry, rz]))


# ─────────────────────────────────────────────────────────────────────────────
# PHARMACY ROOM
# ─────────────────────────────────────────────────────────────────────────────
def build_pharmacy():
    cx = PH_X

    # Floor
    bb.createBody('box', halfExtent=[ROOM_HW, ROOM_HD, 0.04],
                  position=[cx, 0, 0], color='#F2F2F2', mass=0)
    for gx in range(-3, 5):
        bb.createBody('box', halfExtent=[0.007, ROOM_HD, 0.002],
                      position=[cx - ROOM_HW + gx * 1.0 + 0.5, 0, 0.042],
                      color='#CCCCCC', mass=0)
    for gy in range(-3, 4):
        bb.createBody('box', halfExtent=[ROOM_HW, 0.007, 0.002],
                      position=[cx, float(gy), 0.042], color='#CCCCCC', mass=0)
    bb.createBody('box', halfExtent=[0.025, ROOM_HD - 0.1, 0.12],
                  position=[cx - ROOM_HW + 0.13, 0, 0.12], color='#1565C0', mass=0)

    # Left wall
    bb.createBody('box', halfExtent=[0.12, ROOM_HD, H],
                  position=[cx - ROOM_HW, 0, H], color='#FFFFFF', mass=0)
    bb.createBody('box', halfExtent=[0.015, ROOM_HD - 0.2, H - 0.1],
                  position=[cx - ROOM_HW + 0.13, 0, H], color='#E8EAF6', mass=0)

    # Back wall
    bb.createBody('box', halfExtent=[ROOM_HW, 0.12, H],
                  position=[cx, ROOM_HD, H], color='#FFFFFF', mass=0)
    for hz_stripe in [0.5, 1.5, 2.5]:
        bb.createBody('box', halfExtent=[ROOM_HW - 0.2, 0.012, 0.012],
                      position=[cx, ROOM_HD - 0.13, hz_stripe],
                      color='#BBDEFB', mass=0)

    # Bottom wall
    bb.createBody('box', halfExtent=[ROOM_HW, 0.12, H],
                  position=[cx, -ROOM_HD, H], color='#FFFFFF', mass=0)
    for hz_stripe in [0.5, 1.5, 2.5]:
        bb.createBody('box', halfExtent=[ROOM_HW - 0.2, 0.012, 0.012],
                      position=[cx, -ROOM_HD + 0.13, hz_stripe],
                      color='#BBDEFB', mass=0)

    # FIX: Room sign raised well above wall top (H*2 + 0.5) so it's always visible
    bb.createDebugText('PHARMACY',
                       (cx, ROOM_HD - 0.15, H * 2 + 0.5),
                       Q0, color='#1565C0', size=0.42)

    # Automated drug dispensing cabinet
    bb.createBody('box', halfExtent=[0.09, 3.40, 1.40],
                  position=[cx - ROOM_HW + 0.22, 0, 1.40], color='#ECEFF1', mass=0)
    for row in range(6):
        for col in range(-3, 4):
            dz  = 0.25 + row * 0.40
            dy  = col  * 0.44
            bb.createBody('box', halfExtent=[0.055, 0.19, 0.16],
                          position=[cx - ROOM_HW + 0.28, dy, dz],
                          color='#1E88E5', mass=0)
            bb.createBody('box', halfExtent=[0.035, 0.16, 0.12],
                          position=[cx - ROOM_HW + 0.31, dy, dz],
                          color='#42A5F5', mass=0)
            bb.createBody('box', halfExtent=[0.020, 0.07, 0.010],
                          position=[cx - ROOM_HW + 0.33, dy, dz + 0.09],
                          color='#CFD8DC', mass=0)
            bb.createBody('sphere', radius=0.013,
                          position=[cx - ROOM_HW + 0.33, dy + 0.15, dz + 0.09],
                          color='#00C853', mass=0)
    bb.createBody('box', halfExtent=[0.07, 3.35, 0.10],
                  position=[cx - ROOM_HW + 0.25, 0, 2.80], color='#0D47A1', mass=0)
    bb.createBody('box', halfExtent=[0.04, 3.20, 0.06],
                  position=[cx - ROOM_HW + 0.26, 0, 2.80], color='#42A5F5', mass=0)

    # Cold storage fridges
    for fx_off in [-2.6, -0.8, 1.0]:
        bb.createBody('box', halfExtent=[0.52, 0.26, 0.84],
                      position=[cx + fx_off, ROOM_HD - 0.28, 0.86],
                      color='#FAFAFA', mass=0)
        bb.createBody('box', halfExtent=[0.48, 0.020, 0.80],
                      position=[cx + fx_off, ROOM_HD - 0.04, 0.86],
                      color='#B3E5FC', mass=0)
        for sh in [0.35, 0.62, 0.90, 1.18]:
            bb.createBody('box', halfExtent=[0.44, 0.22, 0.010],
                          position=[cx + fx_off, ROOM_HD - 0.18, sh],
                          color='#E3F2FD', mass=0)
        # FIX: deterministic vial colors instead of random
        vial_colors = ['#EF9A9A','#A5D6A7','#FFF59D','#CE93D8','#80DEEA']
        for vi, vc in enumerate(vial_colors):
            bb.createBody('box', halfExtent=[0.022, 0.022, 0.048],
                          position=[cx + fx_off - 0.30 + vi * 0.15,
                                    ROOM_HD - 0.12, 0.40], color=vc, mass=0)
        bb.createBody('box', halfExtent=[0.09, 0.030, 0.050],
                      position=[cx + fx_off + 0.38, ROOM_HD - 0.03, 1.62],
                      color='#212121', mass=0)
        bb.createBody('box', halfExtent=[0.07, 0.012, 0.034],
                      position=[cx + fx_off + 0.38, ROOM_HD - 0.018, 1.62],
                      color='#00BCD4', mass=0)
        bb.createBody('sphere', radius=0.016,
                      position=[cx + fx_off + 0.42, ROOM_HD - 0.018, 1.74],
                      color='#00E676', mass=0)
        bb.createBody('box', halfExtent=[0.010, 0.010, 0.14],
                      position=[cx + fx_off + 0.44, ROOM_HD - 0.038, 0.86],
                      color='#B0BEC5', mass=0)

    # Dispensing counter
    bb.createBody('box', halfExtent=[1.20, 0.50, 0.50],
                  position=[cx + 2.8, 0.30, 0.52], color='#FFFFFF', mass=0)
    bb.createBody('box', halfExtent=[1.18, 0.018, 0.48],
                  position=[cx + 2.8, -0.20, 0.52], color='#1565C0', mass=0)
    bb.createBody('box', halfExtent=[1.20, 0.50, 0.030],
                  position=[cx + 2.8, 0.30, 1.052], color='#ECEFF1', mass=0)
    bb.createBody('box', halfExtent=[1.18, 0.005, 0.005],
                  position=[cx + 2.8, -0.20, 1.060], color='#00E5FF', mass=0)
    bb.createBody('box', halfExtent=[0.36, 0.26, 0.014],
                  position=[cx + 2.8, 0.35, 1.066], color='#0D1B2A', mass=0)
    bb.createBody('box', halfExtent=[0.32, 0.22, 0.006],
                  position=[cx + 2.8, 0.35, 1.072], color='#003366', mass=0)
    for scr_z, scr_c in [(1.082,'#00E676'),(1.076,'#40C4FF'),(1.070,'#FFEB3B')]:
        bb.createBody('box', halfExtent=[0.24, 0.005, 0.004],
                      position=[cx + 2.8, 0.35, scr_z], color=scr_c, mass=0)
    bb.createBody('box', halfExtent=[0.11, 0.09, 0.007],
                  position=[cx + 3.60, 0.10, 1.060], color='#1A237E', mass=0)
    bb.createBody('box', halfExtent=[0.09, 0.07, 0.003],
                  position=[cx + 3.60, 0.10, 1.064], color='#00E5FF', mass=0)
    bb.createBody('box', halfExtent=[0.020, 0.020, 0.30],
                  position=[cx + 3.80, 0.42, 1.35], color='#78909C', mass=0)
    bb.createBody('box', halfExtent=[0.09, 0.020, 0.020],
                  position=[cx + 3.72, 0.42, 1.65], color='#78909C', mass=0)
    bb.createBody('box', halfExtent=[0.060, 0.060, 0.060],
                  position=[cx + 3.65, 0.42, 1.66], color='#263238', mass=0)
    bb.createBody('box', halfExtent=[0.038, 0.038, 0.010],
                  position=[cx + 3.65, 0.39, 1.66], color='#F44336', mass=0)

    # Pharmacist workstation
    bb.createBody('box', halfExtent=[1.55, 0.30, 0.44],
                  position=[cx - 0.60, -ROOM_HD + 0.34, 0.46], color='#FFFFFF', mass=0)
    bb.createBody('box', halfExtent=[1.53, 0.28, 0.028],
                  position=[cx - 0.60, -ROOM_HD + 0.32, 0.910], color='#F5F5F5', mass=0)
    bb.createBody('box', halfExtent=[1.53, 0.015, 0.42],
                  position=[cx - 0.60, -ROOM_HD + 0.056, 0.46], color='#0288D1', mass=0)
    bb.createBody('box', halfExtent=[0.28, 0.040, 0.19],
                  position=[cx - 1.70, -ROOM_HD + 0.15, 1.30], color='#212121', mass=0)
    bb.createBody('box', halfExtent=[0.24, 0.013, 0.16],
                  position=[cx - 1.70, -ROOM_HD + 0.12, 1.30], color='#1565C0', mass=0)
    bb.createBody('box', halfExtent=[0.020, 0.020, 0.19],
                  position=[cx - 1.70, -ROOM_HD + 0.21, 1.02], color='#607D8B', mass=0)
    bb.createBody('box', halfExtent=[0.34, 0.040, 0.21],
                  position=[cx + 0.40, -ROOM_HD + 0.15, 1.32], color='#212121', mass=0)
    bb.createBody('box', halfExtent=[0.30, 0.013, 0.18],
                  position=[cx + 0.40, -ROOM_HD + 0.12, 1.32], color='#2E7D32', mass=0)
    # FIX: deterministic screen line colors
    screen_colors = ['#FFFFFF','#A5D6A7','#80DEEA','#A5D6A7','#FFFFFF']
    for i, scl in enumerate([1.42, 1.35, 1.28, 1.21, 1.14]):
        bb.createBody('box', halfExtent=[0.22, 0.004, 0.004],
                      position=[cx + 0.40, -ROOM_HD + 0.116, scl],
                      color=screen_colors[i], mass=0)
    bb.createBody('box', halfExtent=[0.020, 0.020, 0.21],
                  position=[cx + 0.40, -ROOM_HD + 0.21, 1.02], color='#607D8B', mass=0)
    bb.createBody('box', halfExtent=[0.22, 0.10, 0.011],
                  position=[cx - 0.40, -ROOM_HD + 0.24, 0.924], color='#EEEEEE', mass=0)
    for kr in range(4):
        bb.createBody('box', halfExtent=[0.19, 0.016, 0.005],
                      position=[cx - 0.40, -ROOM_HD + 0.165 + kr * 0.020, 0.937],
                      color='#BDBDBD', mass=0)
    bb.createBody('sphere', radius=0.030,
                  position=[cx + 0.56, -ROOM_HD + 0.21, 0.938], color='#EEEEEE', mass=0)

    # UV sterilisation cabinet
    bb.createBody('box', halfExtent=[0.40, 0.28, 0.60],
                  position=[cx - 2.80, -ROOM_HD + 0.30, 0.62], color='#FAFAFA', mass=0)
    bb.createBody('box', halfExtent=[0.36, 0.018, 0.56],
                  position=[cx - 2.80, -ROOM_HD + 0.04, 0.62], color='#CE93D8', mass=0)
    bb.createBody('box', halfExtent=[0.016, 0.22, 0.016],
                  position=[cx - 2.80, -ROOM_HD + 0.18, 0.78], color='#7B1FA2', mass=0)
    bb.createBody('box', halfExtent=[0.016, 0.22, 0.016],
                  position=[cx - 2.80, -ROOM_HD + 0.18, 0.50], color='#7B1FA2', mass=0)
    bb.createBody('box', halfExtent=[0.033, 0.075, 0.09],
                  position=[cx - 2.42, -ROOM_HD + 0.28, 1.06], color='#263238', mass=0)
    bb.createBody('sphere', radius=0.018,
                  position=[cx - 2.42, -ROOM_HD + 0.23, 1.20], color='#00E676', mass=0)

    # Ceiling robotic pill-count arm
    bb.createBody('box', halfExtent=[0.11, 0.11, 0.052],
                  position=[cx + 1.4, 2.0, H * 2 - 0.06], color='#CFD8DC', mass=0)
    bb.createBody('box', halfExtent=[0.036, 0.036, 0.50],
                  position=[cx + 1.4, 2.0, H * 2 - 0.58], color='#90A4AE', mass=0)
    bb.createBody('sphere', radius=0.072,
                  position=[cx + 1.4, 2.0, H * 2 - 1.10], color='#1565C0', mass=0)
    bb.createBody('box', halfExtent=[0.028, 0.28, 0.028],
                  position=[cx + 1.4, 1.72, H * 2 - 1.14], color='#CFD8DC', mass=0)
    bb.createBody('sphere', radius=0.052,
                  position=[cx + 1.4, 1.44, H * 2 - 1.15], color='#0288D1', mass=0)
    bb.createBody('box', halfExtent=[0.024, 0.20, 0.024],
                  position=[cx + 1.4, 1.24, H * 2 - 1.18], color='#CFD8DC', mass=0)
    bb.createBody('box', halfExtent=[0.052, 0.052, 0.036],
                  position=[cx + 1.4, 1.04, H * 2 - 1.22], color='#37474F', mass=0)
    bb.createBody('box', halfExtent=[0.010, 0.010, 0.052],
                  position=[cx + 1.30, 1.04, H * 2 - 1.27], color='#1565C0', mass=0)
    bb.createBody('box', halfExtent=[0.010, 0.010, 0.052],
                  position=[cx + 1.50, 1.04, H * 2 - 1.27], color='#1565C0', mass=0)

    # Prescription display board
    bb.createBody('box', halfExtent=[0.88, 0.052, 0.58],
                  position=[cx + 2.0, ROOM_HD - 0.055, 2.14], color='#212121', mass=0)
    bb.createBody('box', halfExtent=[0.83, 0.017, 0.54],
                  position=[cx + 2.0, ROOM_HD - 0.022, 2.14], color='#003366', mass=0)
    for drow, dcol in [(2.46,'#00E676'),(2.34,'#40C4FF'),(2.22,'#FFEB3B'),
                       (2.10,'#FFFFFF'),(1.98,'#E0E0E0'),(1.86,'#FF5252')]:
        bb.createBody('box', halfExtent=[0.64, 0.006, 0.006],
                      position=[cx + 2.0, ROOM_HD - 0.016, drow], color=dcol, mass=0)

    # Robot charging dock
    bb.createBody('box', halfExtent=[0.48, 0.32, 0.025],
                  position=[DOCK_X, DOCK_Y, 0.025], color='#E3F2FD', mass=0)
    bb.createBody('box', halfExtent=[0.46, 0.30, 0.011],
                  position=[DOCK_X, DOCK_Y, 0.036], color='#42A5F5', mass=0)
    for dang in range(0, 360, 45):
        ddx = 0.19 * math.cos(math.radians(dang))
        ddy = 0.19 * math.sin(math.radians(dang))
        bb.createBody('sphere', radius=0.016,
                      position=[DOCK_X + ddx, DOCK_Y + ddy, 0.044],
                      color='#00E676', mass=0)
    bb.createBody('box', halfExtent=[0.034, 0.034, 0.55],
                  position=[DOCK_X, DOCK_Y - 0.29, 0.58], color='#B0BEC5', mass=0)
    bb.createBody('box', halfExtent=[0.13, 0.019, 0.10],
                  position=[DOCK_X, DOCK_Y - 0.27, 0.80], color='#212121', mass=0)
    bb.createBody('box', halfExtent=[0.10, 0.008, 0.07],
                  position=[DOCK_X, DOCK_Y - 0.262, 0.80], color='#42A5F5', mass=0)

    print("[PHARMACY] Built.")


# ─────────────────────────────────────────────────────────────────────────────
# SHARED DIVIDING WALL WITH DOOR ENTRANCE
# ─────────────────────────────────────────────────────────────────────────────
def build_shared_wall():
    wx = WALL_X

    top_half  = (ROOM_HD - DOOR_W) / 2.0
    top_cen_y =  DOOR_W + top_half
    bb.createBody('box', halfExtent=[WALL_T, top_half, H],
                  position=[wx, top_cen_y, H], color='#FFFFFF', mass=0)
    bb.createBody('box', halfExtent=[0.014, top_half - 0.1, H - 0.1],
                  position=[wx + 0.13, top_cen_y, H], color='#BBDEFB', mass=0)

    bot_half  = (ROOM_HD - DOOR_W) / 2.0
    bot_cen_y = -DOOR_W - bot_half
    bb.createBody('box', halfExtent=[WALL_T, bot_half, H],
                  position=[wx, bot_cen_y, H], color='#FFFFFF', mass=0)
    bb.createBody('box', halfExtent=[0.014, bot_half - 0.1, H - 0.1],
                  position=[wx + 0.13, bot_cen_y, H], color='#BBDEFB', mass=0)

    for sign in [+1, -1]:
        pillar_y = sign * DOOR_W
        bb.createBody('box', halfExtent=[0.14, 0.10, H],
                      position=[wx, pillar_y, H], color='#E0E0E0', mass=0)
        bb.createBody('box', halfExtent=[0.10, 0.04, H - 0.12],
                      position=[wx, pillar_y, H], color='#1565C0', mass=0)

    bb.createBody('box', halfExtent=[0.14, DOOR_W, 0.16],
                  position=[wx, 0, H * 2 - 0.16], color='#E0E0E0', mass=0)
    bb.createBody('box', halfExtent=[0.10, DOOR_W - 0.05, 0.10],
                  position=[wx, 0, H * 2 - 0.18], color='#42A5F5', mass=0)

    bb.createBody('box', halfExtent=[0.055, 0.038, 0.040],
                  position=[wx - 0.06, 0, H * 2 - 0.38], color='#212121', mass=0)
    bb.createBody('sphere', radius=0.016,
                  position=[wx - 0.06, 0, H * 2 - 0.42], color='#00E676', mass=0)
    bb.createBody('box', halfExtent=[0.055, 0.038, 0.040],
                  position=[wx + 0.06, 0, H * 2 - 0.38], color='#212121', mass=0)
    bb.createBody('sphere', radius=0.016,
                  position=[wx + 0.06, 0, H * 2 - 0.42], color='#00E676', mass=0)

    # Floor guide dashes through door
    for sx in [-1.5, -0.8, -0.1, 0.6, 1.3]:
        bb.createBody('box', halfExtent=[0.22, 0.035, 0.003],
                      position=[wx + sx, 0, 0.046], color='#FDD835', mass=0)

    print("[SHARED WALL] Built at X =", wx)


# ─────────────────────────────────────────────────────────────────────────────
# PATIENT ROOM
# ─────────────────────────────────────────────────────────────────────────────
def build_patient_room():
    cx = PT_X

    # Floor
    bb.createBody('box', halfExtent=[ROOM_HW, ROOM_HD, 0.04],
                  position=[cx, 0, 0], color='#FFF8F0', mass=0)
    for gx in range(-3, 5):
        bb.createBody('box', halfExtent=[0.007, ROOM_HD, 0.002],
                      position=[cx - ROOM_HW + gx * 1.0 + 0.5, 0, 0.042],
                      color='#E0D8D0', mass=0)
    for gy in range(-3, 4):
        bb.createBody('box', halfExtent=[ROOM_HW, 0.007, 0.002],
                      position=[cx, float(gy), 0.042], color='#E0D8D0', mass=0)
    bb.createBody('box', halfExtent=[0.025, ROOM_HD - 0.1, 0.12],
                  position=[cx + ROOM_HW - 0.13, 0, 0.12], color='#2E7D32', mass=0)

    # Right wall
    bb.createBody('box', halfExtent=[0.12, ROOM_HD, H],
                  position=[cx + ROOM_HW, 0, H], color='#FFFFFF', mass=0)
    bb.createBody('box', halfExtent=[0.015, ROOM_HD - 0.2, H - 0.1],
                  position=[cx + ROOM_HW - 0.13, 0, H], color='#E8F5E9', mass=0)

    # Back wall
    bb.createBody('box', halfExtent=[ROOM_HW, 0.12, H],
                  position=[cx, ROOM_HD, H], color='#FFFFFF', mass=0)
    for hz_s in [0.5, 1.5, 2.5]:
        bb.createBody('box', halfExtent=[ROOM_HW - 0.2, 0.012, 0.012],
                      position=[cx, ROOM_HD - 0.13, hz_s], color='#C8E6C9', mass=0)

    # Bottom wall
    bb.createBody('box', halfExtent=[ROOM_HW, 0.12, H],
                  position=[cx, -ROOM_HD, H], color='#FFFFFF', mass=0)
    for hz_s in [0.5, 1.5, 2.5]:
        bb.createBody('box', halfExtent=[ROOM_HW - 0.2, 0.012, 0.012],
                      position=[cx, -ROOM_HD + 0.13, hz_s], color='#C8E6C9', mass=0)

    # FIX: Room sign raised well above wall top
    bb.createDebugText('PATIENT ROOM',
                       (cx, ROOM_HD - 0.15, H * 2 + 0.5),
                       Q0, color='#2E7D32', size=0.40)

    bx = cx - 1.8

    # Hospital bed
    bb.createBody('box', halfExtent=[0.56, 1.15, 0.055],
                  position=[bx, 0.35, 0.30], color='#ECEFF1', mass=0)
    for lxo, lyo in [(-0.50,-0.74),(0.50,-0.74),(-0.50,1.50),(0.50,1.50)]:
        bb.createBody('box', halfExtent=[0.052, 0.052, 0.26],
                      position=[bx + lxo, 0.35 + lyo, 0.19], color='#B0BEC5', mass=0)
        bb.createBody('sphere', radius=0.052,
                      position=[bx + lxo, 0.35 + lyo, 0.052], color='#424242', mass=0)
    bb.createBody('box', halfExtent=[0.020, 1.12, 0.17],
                  position=[bx - 0.50, 0.35, 0.55], color='#CFD8DC', mass=0)
    bb.createBody('box', halfExtent=[0.020, 1.12, 0.17],
                  position=[bx + 0.50, 0.35, 0.55], color='#CFD8DC', mass=0)
    bb.createBody('box', halfExtent=[0.48, 1.08, 0.095],
                  position=[bx, 0.35, 0.93], color='#90CAF9', mass=0)
    bb.createBody('box', halfExtent=[0.50, 0.042, 0.40],
                  position=[bx, 1.54, 1.10], color='#ECEFF1', mass=0)
    bb.createBody('box', halfExtent=[0.46, 0.016, 0.36],
                  position=[bx, 1.538, 1.10], color='#1565C0', mass=0)
    bb.createBody('box', halfExtent=[0.50, 0.042, 0.25],
                  position=[bx, -0.76, 1.04], color='#ECEFF1', mass=0)
    bb.createBody('box', halfExtent=[0.17, 0.020, 0.11],
                  position=[bx, 1.525, 1.30], color='#212121', mass=0)
    bb.createBody('box', halfExtent=[0.13, 0.007, 0.08],
                  position=[bx, 1.518, 1.30], color='#00BCD4', mass=0)

    # Patient
    bb.createBody('box', halfExtent=[0.18, 0.56, 0.082],
                  position=[bx, 0.30, 1.10], color='#BBDEFB', mass=0)
    bb.createBody('box', halfExtent=[0.053, 0.40, 0.048],
                  position=[bx - 0.46, 0.18, 1.06], color='#FFCCBC', mass=0)
    bb.createBody('box', halfExtent=[0.053, 0.40, 0.048],
                  position=[bx + 0.46, 0.18, 1.06], color='#FFCCBC', mass=0)
    bb.createBody('sphere', radius=0.115,
                  position=[bx, 1.37, 1.18], color='#FFCCBC', mass=0)
    bb.createBody('box', halfExtent=[0.110, 0.082, 0.038],
                  position=[bx, 1.37, 1.28], color='#4E342E', mass=0)
    bb.createBody('box', halfExtent=[0.072, 0.062, 0.028],
                  position=[bx, 1.355, 1.16], color='#B2EBF2', mass=0)
    for lx5, ly5, lc5 in [(bx-0.13,0.70,'#FF5252'),(bx+0.03,0.70,'#FFEB3B'),
                           (bx-0.13,0.53,'#69F0AE'),(bx+0.03,0.53,'#40C4FF')]:
        bb.createBody('sphere', radius=0.021,
                      position=[lx5, ly5, 1.20], color=lc5, mass=0)

    # Overhead surgical light
    bb.createBody('box', halfExtent=[0.075, 0.075, 0.042],
                  position=[bx, 0.35, H * 2 - 0.06], color='#BDBDBD', mass=0)
    bb.createBody('box', halfExtent=[0.040, 0.040, 0.40],
                  position=[bx, 0.35, H * 2 - 0.50], color='#9E9E9E', mass=0)
    bb.createBody('sphere', radius=0.068,
                  position=[bx, 0.35, H * 2 - 0.93], color='#757575', mass=0)
    bb.createBody('box', halfExtent=[0.040, 0.32, 0.040],
                  position=[bx, 0.03, H * 2 - 0.97], color='#9E9E9E', mass=0)
    bb.createBody('box', halfExtent=[0.34, 0.34, 0.063],
                  position=[bx, -0.33, H * 2 - 1.03], color='#EEEEEE', mass=0)
    bb.createBody('box', halfExtent=[0.30, 0.30, 0.016],
                  position=[bx, -0.33, H * 2 - 1.10], color='#FFFDE7', mass=0)
    for sang in range(0, 360, 30):
        bb.createBody('sphere', radius=0.021,
                      position=[bx + 0.25*math.cos(math.radians(sang)),
                                 -0.33 + 0.25*math.sin(math.radians(sang)),
                                 H * 2 - 1.12], color='#FFFDE7', mass=0)

    # Vital signs monitor
    bb.createBody('box', halfExtent=[0.048, 0.048, 0.65],
                  position=[cx + ROOM_HW - 0.14, 1.30, H * 2 - 0.70], color='#607D8B', mass=0)
    bb.createBody('box', halfExtent=[0.42, 0.048, 0.040],
                  position=[cx + ROOM_HW - 0.56, 1.30, H * 2 - 1.40], color='#78909C', mass=0)
    bb.createBody('box', halfExtent=[0.28, 0.075, 0.22],
                  position=[cx + ROOM_HW - 1.00, 1.30, H * 2 - 1.56], color='#212121', mass=0)
    bb.createBody('box', halfExtent=[0.24, 0.023, 0.18],
                  position=[cx + ROOM_HW - 1.00, 1.277, H * 2 - 1.56], color='#0D47A1', mass=0)
    for wli, wlc in enumerate(['#00E676','#FF5252','#FFEB3B','#40C4FF']):
        bb.createBody('box', halfExtent=[0.20, 0.005, 0.006],
                      position=[cx + ROOM_HW - 1.00, 1.272,
                                 H * 2 - 1.47 + wli * 0.076], color=wlc, mass=0)

    # FIX: IV drip stand moved slightly away from robot approach path
    # was bx+0.88 — moved to bx+1.10 so it doesn't clip robot walking to bedside
    iv_x = bx + 1.10
    iv_y = 1.68
    bb.createBody('box', halfExtent=[0.020, 0.020, 1.42],
                  position=[iv_x, iv_y, 1.42], color='#B0BEC5', mass=0)
    bb.createBody('sphere', radius=0.038,
                  position=[iv_x, iv_y, 0.038], color='#424242', mass=0)
    bb.createBody('box', halfExtent=[0.09, 0.010, 0.010],
                  position=[iv_x, iv_y, 2.83], color='#B0BEC5', mass=0)
    for bof, bcol in [(0.0,'#E3F2FD'),(-0.20,'#FFF9C4'),(0.20,'#FCE4EC')]:
        bb.createBody('box', halfExtent=[0.052, 0.020, 0.090],
                      position=[iv_x + bof, iv_y, 2.73], color=bcol, mass=0)
        bb.createBody('box', halfExtent=[0.005, 0.005, 0.52],
                      position=[iv_x + bof, iv_y, 2.17], color='#E0E0E0', mass=0)
    bb.createBody('box', halfExtent=[0.095, 0.062, 0.088],
                  position=[iv_x, iv_y - 0.062, 1.82], color='#1565C0', mass=0)
    bb.createBody('box', halfExtent=[0.070, 0.017, 0.062],
                  position=[iv_x, iv_y - 0.078, 1.82], color='#E3F2FD', mass=0)

    # Bedside table
    bb.createBody('box', halfExtent=[0.038, 0.038, 0.52],
                  position=[BEDSIDE_X - 0.48, BEDSIDE_Y, 0.54], color='#90A4AE', mass=0)
    bb.createBody('box', halfExtent=[0.28, 0.028, 0.028],
                  position=[BEDSIDE_X - 0.20, BEDSIDE_Y, 1.06], color='#90A4AE', mass=0)
    bb.createBody('box', halfExtent=[0.34, 0.28, 0.026],
                  position=[BEDSIDE_X, BEDSIDE_Y, BEDSIDE_Z - 0.038], color='#ECEFF1', mass=0)
    bb.createBody('box', halfExtent=[0.34, 0.020, 0.028],
                  position=[BEDSIDE_X, BEDSIDE_Y - 0.29, BEDSIDE_Z - 0.020], color='#78909C', mass=0)
    bb.createBody('box', halfExtent=[0.34, 0.020, 0.028],
                  position=[BEDSIDE_X, BEDSIDE_Y + 0.26, BEDSIDE_Z - 0.020], color='#78909C', mass=0)
    bb.createBody('box', halfExtent=[0.42, 0.065, 0.035],
                  position=[BEDSIDE_X - 0.22, BEDSIDE_Y, 0.035], color='#B0BEC5', mass=0)
    for wx6, wy6 in [(BEDSIDE_X - 0.48, BEDSIDE_Y),(BEDSIDE_X + 0.04, BEDSIDE_Y)]:
        bb.createBody('sphere', radius=0.048,
                      position=[wx6, wy6, 0.048], color='#424242', mass=0)
    bb.createBody('box', halfExtent=[0.11, 0.09, 0.013],
                  position=[BEDSIDE_X, BEDSIDE_Y, BEDSIDE_Z + 0.004], color='#B3E5FC', mass=0)

    # Ventilator
    bb.createBody('box', halfExtent=[0.21, 0.24, 0.53],
                  position=[cx + ROOM_HW - 0.23, -2.10, 0.55], color='#FAFAFA', mass=0)
    bb.createBody('box', halfExtent=[0.17, 0.055, 0.22],
                  position=[cx + ROOM_HW - 0.23, -1.87, 0.78], color='#1565C0', mass=0)
    bb.createBody('box', halfExtent=[0.13, 0.022, 0.18],
                  position=[cx + ROOM_HW - 0.23, -1.848, 0.78], color='#42A5F5', mass=0)
    # FIX: deterministic ventilator display colors
    vent_colors = ['#00E676','#FFEB3B','#FF5252','#00E676']
    for vi, vdz in enumerate([0.90, 0.82, 0.74, 0.66]):
        bb.createBody('box', halfExtent=[0.10, 0.005, 0.005],
                      position=[cx + ROOM_HW - 0.23, -1.843, vdz],
                      color=vent_colors[vi], mass=0)
    for vwx2, vwy2 in [(cx+ROOM_HW-0.42,-2.28),(cx+ROOM_HW-0.04,-2.28),
                        (cx+ROOM_HW-0.42,-1.92),(cx+ROOM_HW-0.04,-1.92)]:
        bb.createBody('sphere', radius=0.052,
                      position=[vwx2, vwy2, 0.052], color='#424242', mass=0)

    # Crash cart
    bb.createBody('box', halfExtent=[0.21, 0.33, 0.52],
                  position=[cx + ROOM_HW - 0.23, -3.20, 0.54], color='#EF5350', mass=0)
    for dz2 in [0.20, 0.36, 0.52, 0.68, 0.84]:
        bb.createBody('box', halfExtent=[0.19, 0.31, 0.042],
                      position=[cx + ROOM_HW - 0.23, -3.20, dz2 + 0.028], color='#E53935', mass=0)
        bb.createBody('sphere', radius=0.021,
                      position=[cx + ROOM_HW - 0.05, -3.20, dz2 + 0.034], color='#FDD835', mass=0)
    bb.createBody('box', halfExtent=[0.17, 0.21, 0.13],
                  position=[cx + ROOM_HW - 0.23, -3.20, 1.12], color='#212121', mass=0)
    bb.createBody('box', halfExtent=[0.12, 0.15, 0.025],
                  position=[cx + ROOM_HW - 0.23, -3.20, 1.28], color='#F57F17', mass=0)
    for cwx2, cwy2 in [(cx+ROOM_HW-0.42,-3.40),(cx+ROOM_HW-0.04,-3.40),
                        (cx+ROOM_HW-0.42,-3.00),(cx+ROOM_HW-0.04,-3.00)]:
        bb.createBody('sphere', radius=0.052,
                      position=[cwx2, cwy2, 0.052], color='#424242', mass=0)

    # Nurse call panel
    bb.createBody('box', halfExtent=[0.14, 0.052, 0.26],
                  position=[bx - 0.80, ROOM_HD - 0.065, 1.46], color='#ECEFF1', mass=0)
    bb.createBody('box', halfExtent=[0.12, 0.020, 0.24],
                  position=[bx - 0.80, ROOM_HD - 0.038, 1.46], color='#0D47A1', mass=0)
    bb.createBody('sphere', radius=0.058,
                  position=[bx - 0.80, ROOM_HD - 0.038, 1.66], color='#F44336', mass=0)
    bb.createBody('sphere', radius=0.040,
                  position=[bx - 0.80, ROOM_HD - 0.038, 1.42], color='#4CAF50', mass=0)
    bb.createBody('sphere', radius=0.028,
                  position=[bx - 0.80, ROOM_HD - 0.038, 1.23], color='#FFEB3B', mass=0)

    # Wall digital display
    bb.createBody('box', halfExtent=[0.92, 0.052, 0.60],
                  position=[cx + 2.40, ROOM_HD - 0.055, 2.18], color='#212121', mass=0)
    bb.createBody('box', halfExtent=[0.87, 0.017, 0.55],
                  position=[cx + 2.40, ROOM_HD - 0.022, 2.18], color='#003366', mass=0)
    for drow2, dcol2 in [(2.52,'#00E676'),(2.40,'#40C4FF'),(2.28,'#FFEB3B'),
                          (2.16,'#FFFFFF'),(2.04,'#E0E0E0'),(1.92,'#FF5252')]:
        bb.createBody('box', halfExtent=[0.68, 0.006, 0.006],
                      position=[cx + 2.40, ROOM_HD - 0.016, drow2], color=dcol2, mass=0)

    # Waste bins
    for bxb, byb, bcb in [(cx+2.80,-ROOM_HD+0.22,'#FDD835'),
                           (cx+3.20,-ROOM_HD+0.22,'#EF5350'),
                           (cx+3.60,-ROOM_HD+0.22,'#42A5F5')]:
        bb.createBody('box', halfExtent=[0.13, 0.13, 0.26],
                      position=[bxb, byb, 0.26], color=bcb, mass=0)
        bb.createBody('box', halfExtent=[0.14, 0.14, 0.036],
                      position=[bxb, byb, 0.542], color='#FAFAFA', mass=0)
        bb.createBody('sphere', radius=0.052,
                      position=[bxb, byb - 0.05, 0.60], color='#EEEEEE', mass=0)
        bb.createBody('sphere', radius=0.016,
                      position=[bxb, byb - 0.08, 0.665], color='#00E5FF', mass=0)

    print("[PATIENT ROOM] Built.")


# ─────────────────────────────────────────────────────────────────────────────
# TABLET
# ─────────────────────────────────────────────────────────────────────────────
class Tablet:
    def __init__(self):
        self.bottle = bb.createBody('box', halfExtent=[0.028, 0.028, 0.065],
                                    position=[TABLET_X, TABLET_Y, TABLET_Z + 0.065],
                                    color='#FF8F00', mass=0)
        self.cap    = bb.createBody('box', halfExtent=[0.031, 0.031, 0.017],
                                    position=[TABLET_X, TABLET_Y, TABLET_Z + 0.147],
                                    color='#FFFFFF', mass=0)
        self.label  = bb.createBody('box', halfExtent=[0.029, 0.010, 0.025],
                                    position=[TABLET_X, TABLET_Y - 0.019, TABLET_Z + 0.075],
                                    color='#1565C0', mass=0)
        self.rfid   = bb.createBody('sphere', radius=0.009,
                                    position=[TABLET_X, TABLET_Y, TABLET_Z + 0.004],
                                    color='#00E5FF', mass=0)

    def move_to(self, x, y, z, yaw=0.0):
        q = bb.getQuaternionFromEuler([0, 0, yaw])
        bb.resetBasePose(self.bottle, [x, y, z + 0.065], q)
        bb.resetBasePose(self.cap,    [x, y, z + 0.147], q)
        bb.resetBasePose(self.label,  [x, y - 0.019, z + 0.075], q)
        bb.resetBasePose(self.rfid,   [x, y, z + 0.004], q)

    def rest_on_counter(self):
        self.move_to(TABLET_X, TABLET_Y, TABLET_Z)

    def rest_on_bedside(self):
        self.move_to(BEDSIDE_X, BEDSIDE_Y, BEDSIDE_Z)


# ─────────────────────────────────────────────────────────────────────────────
# ROBOT
# ─────────────────────────────────────────────────────────────────────────────
class MedicalRobot:
    BASE_H    = 0.42
    COL_H     = 0.22
    UPPER_LEN = 0.56
    LOWER_LEN = 0.48
    WRIST_LEN = 0.20

    FINGER_OPEN   = 0.10
    FINGER_CLOSED = 0.024

    # Manual drive speed constants
    DRIVE_SPEED = 0.06   # metres per key press
    TURN_SPEED  = 0.08   # radians per key press
    ARM_SPEED   = 0.06   # radians per key press

    def __init__(self):
        self.x = DOCK_X; self.y = DOCK_Y; self.z = 0.0
        self.yaw = 0.0
        self.arm_yaw = 0.0
        self.shoulder_pitch = -1.2
        self.elbow_flex = 2.1
        self.wrist_pitch = 0.0
        self.gripper = 0.0
        self.wt = 0.0
        self.ids = {}
        self.hand_wx = self.hand_wy = self.hand_wz = 0.0

    def _mk(self, k, shape, **kw):
        kw['mass'] = 0
        self.ids[k] = bb.createBody(shape, **kw)

    def spawn(self):
        d = [0, 0, -20]
        self._mk('base_lo',    'box',    halfExtent=[0.34, 0.40, 0.065], position=d, color='#1A237E')
        self._mk('base_mid',   'box',    halfExtent=[0.32, 0.38, 0.105], position=d, color='#283593')
        self._mk('base_top',   'box',    halfExtent=[0.32, 0.38, 0.015], position=d, color='#3F51B5')
        self._mk('base_panel', 'box',    halfExtent=[0.30, 0.36, 0.010], position=d, color='#E8EAF6')
        self._mk('cross_h',    'box',    halfExtent=[0.11, 0.026, 0.007], position=d, color='#F44336')
        self._mk('cross_v',    'box',    halfExtent=[0.026, 0.11, 0.007], position=d, color='#F44336')
        self._mk('bumper_f',   'box',    halfExtent=[0.32, 0.022, 0.044], position=d, color='#FF6F00')
        self._mk('bumper_b',   'box',    halfExtent=[0.32, 0.022, 0.044], position=d, color='#FF6F00')
        self._mk('bumper_l',   'box',    halfExtent=[0.022, 0.38, 0.044], position=d, color='#FF6F00')
        self._mk('bumper_r',   'box',    halfExtent=[0.022, 0.38, 0.044], position=d, color='#FF6F00')
        for k in ['prox_fl', 'prox_fc', 'prox_fr']:
            self._mk(k, 'sphere', radius=0.024, position=d, color='#00BCD4')
        self._mk('lidar_lo',   'box',    halfExtent=[0.28, 0.28, 0.030], position=d, color='#1A237E')
        self._mk('lidar_lo_l', 'box',    halfExtent=[0.26, 0.26, 0.015], position=d, color='#00E676')
        self._mk('lidar_hi',   'box',    halfExtent=[0.22, 0.22, 0.024], position=d, color='#283593')
        self._mk('lidar_hi_l', 'box',    halfExtent=[0.20, 0.20, 0.011], position=d, color='#69F0AE')
        self._mk('scan_1',     'sphere', radius=0.034, position=d, color='#F44336')
        self._mk('scan_2',     'sphere', radius=0.034, position=d, color='#F44336')
        for k in ['w_fl','w_fr','w_bl','w_br']:
            self._mk(k,        'sphere', radius=0.098,                   position=d, color='#212121')
            self._mk(k+'_h',   'box',    halfExtent=[0.048,0.048,0.098], position=d, color='#424242')
            self._mk(k+'_r',   'box',    halfExtent=[0.052,0.052,0.014], position=d, color='#3F51B5')
        self._mk('col_lo',  'box',    halfExtent=[0.12, 0.12, 0.085], position=d, color='#37474F')
        self._mk('col_hi',  'box',    halfExtent=[0.09, 0.09, 0.065], position=d, color='#455A64')
        self._mk('col_top', 'sphere', radius=0.095,                   position=d, color='#546E7A')
        self._mk('j0',        'sphere', radius=0.105,                  position=d, color='#1E88E5')
        self._mk('j0_ring',   'box',    halfExtent=[0.13,0.13,0.018],  position=d, color='#1565C0')
        self._mk('ua_link',   'box',    halfExtent=[0.054,0.054,0.28], position=d, color='#ECEFF1')
        self._mk('ua_stripe', 'box',    halfExtent=[0.011,0.056,0.26], position=d, color='#1E88E5')
        self._mk('ua_cover',  'box',    halfExtent=[0.052,0.052,0.030],position=d, color='#1E88E5')
        self._mk('j2',        'sphere', radius=0.080,                  position=d, color='#1E88E5')
        self._mk('j2_ring',   'box',    halfExtent=[0.094,0.094,0.015],position=d, color='#0288D1')
        self._mk('fa_link',   'box',    halfExtent=[0.046,0.046,0.24], position=d, color='#ECEFF1')
        self._mk('fa_stripe', 'box',    halfExtent=[0.011,0.048,0.22], position=d, color='#0288D1')
        self._mk('fa_cover',  'box',    halfExtent=[0.044,0.044,0.025],position=d, color='#0288D1')
        self._mk('j4',        'sphere', radius=0.065,                  position=d, color='#0288D1')
        self._mk('wr_link',   'box',    halfExtent=[0.040,0.040,0.098],position=d, color='#ECEFF1')
        self._mk('j6',        'sphere', radius=0.055,                  position=d, color='#0288D1')
        self._mk('flange',    'box',    halfExtent=[0.056,0.056,0.022],position=d, color='#37474F')
        self._mk('ft',        'box',    halfExtent=[0.050,0.050,0.026],position=d, color='#FF8F00')
        self._mk('g_palm',  'box',    halfExtent=[0.068,0.030,0.068], position=d, color='#37474F')
        self._mk('g_fa_u',  'box',    halfExtent=[0.014,0.012,0.088], position=d, color='#455A64')
        self._mk('g_fa_l',  'box',    halfExtent=[0.012,0.012,0.042], position=d, color='#263238')
        self._mk('g_fb_u',  'box',    halfExtent=[0.014,0.012,0.088], position=d, color='#455A64')
        self._mk('g_fb_l',  'box',    halfExtent=[0.012,0.012,0.042], position=d, color='#263238')
        self._mk('g_pad_a', 'box',    halfExtent=[0.009,0.016,0.034], position=d, color='#212121')
        self._mk('g_pad_b', 'box',    halfExtent=[0.009,0.016,0.034], position=d, color='#212121')
        self._mk('g_tip_a', 'sphere', radius=0.015,                   position=d, color='#455A64')
        self._mk('g_tip_b', 'sphere', radius=0.015,                   position=d, color='#455A64')
        self._mk('cam',    'box',    halfExtent=[0.058,0.015,0.019], position=d, color='#212121')
        self._mk('cam_l',  'sphere', radius=0.010,                   position=d, color='#B71C1C')
        self._mk('cam_r',  'sphere', radius=0.010,                   position=d, color='#1A237E')
        self._mk('cam_ir', 'sphere', radius=0.008,                   position=d, color='#880000')
        self.update()
        print("[ROBOT] Spawned at dock.")

    def _w(self, lx, ly, lz):
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        return (self.x + cy*lx - sy*ly,
                self.y + sy*lx + cy*ly,
                self.z + lz)

    def _p(self, key, lx, ly, lz, rx=0, ry=0, rz=None):
        wx, wy, wz = self._w(lx, ly, lz)
        rz = rz if rz is not None else self.yaw
        place(self.ids[key], wx, wy, wz, rx, ry, rz)

    def _arm_fk(self):
        ay, sp, ef, wp = self.arm_yaw, self.shoulder_pitch, self.elbow_flex, self.wrist_pitch
        sx, sy, sz = self._w(0, 0, self.BASE_H + self.COL_H)
        cay, say = math.cos(ay), math.sin(ay)
        csp, ssp = math.cos(sp), math.sin(sp)
        UL = self.UPPER_LEN
        ex = sx + cay*csp*UL; ey = sy + say*csp*UL; ez = sz + ssp*UL
        tp = sp + ef; ctp, stp = math.cos(tp), math.sin(tp)
        FL = self.LOWER_LEN
        wx2 = ex + cay*ctp*FL; wy2 = ey + say*ctp*FL; wz2 = ez + stp*FL
        wrp = tp + wp; cwp, swp = math.cos(wrp), math.sin(wrp)
        WL = self.WRIST_LEN
        hx = wx2 + cay*cwp*WL; hy = wy2 + say*cwp*WL; hz = wz2 + swp*WL
        return (sx,sy,sz),(ex,ey,ez),(wx2,wy2,wz2),(hx,hy,hz),ay,sp,ef,wp

    def update(self):
        wt = self.wt
        self._p('base_lo',    0, 0, 0.075)
        self._p('base_mid',   0, 0, 0.182)
        self._p('base_top',   0, 0, 0.298)
        self._p('base_panel', 0, 0, 0.311)
        self._p('cross_h',    0, 0, 0.319)
        self._p('cross_v',    0, 0, 0.319)
        self._p('bumper_f',  0,  0.38, 0.18)
        self._p('bumper_b',  0, -0.38, 0.18)
        self._p('bumper_l', -0.32, 0,  0.18)
        self._p('bumper_r',  0.32, 0,  0.18)
        self._p('prox_fl',  -0.24,  0.39, 0.18)
        self._p('prox_fc',   0.00,  0.39, 0.18)
        self._p('prox_fr',   0.24,  0.39, 0.18)
        self._p('lidar_lo',   0, 0, 0.332)
        self._p('lidar_lo_l', 0, 0, 0.350)
        self._p('lidar_hi',   0, 0, 0.370)
        self._p('lidar_hi_l', 0, 0, 0.384)
        sr = 0.22; sa = wt * 3.2
        self._p('scan_1', sr*math.cos(sa),        sr*math.sin(sa),        0.382)
        self._p('scan_2', sr*math.cos(sa+math.pi),sr*math.sin(sa+math.pi),0.350)
        wo = [(-0.29,-0.34),(0.29,-0.34),(-0.29,0.34),(0.29,0.34)]
        wk = ['w_fl','w_fr','w_bl','w_br']; rp = [0,math.pi,math.pi,0]
        for (lx2,ly2),k,rph in zip(wo,wk,rp):
            wx7,wy7,wz7 = self._w(lx2,ly2,0.098)
            roll = math.sin(wt+rph)*0.38
            place(self.ids[k],      wx7,wy7,wz7,roll,0,self.yaw)
            place(self.ids[k+'_h'], wx7,wy7,wz7,0,   0,self.yaw)
            place(self.ids[k+'_r'], wx7,wy7,wz7,0,   0,self.yaw)
        self._p('col_lo',  0, 0, self.BASE_H - 0.065)
        self._p('col_hi',  0, 0, self.BASE_H + 0.065)
        self._p('col_top', 0, 0, self.BASE_H + 0.178)
        (sx,sy,sz),(ex,ey,ez),(wx2,wy2,wz2),(hx,hy,hz),ay,sp,ef,wp = self._arm_fk()
        self.hand_wx=hx; self.hand_wy=hy; self.hand_wz=hz
        arm_rz = ay
        place(self.ids['j0'],      sx,sy,sz)
        place(self.ids['j0_ring'], sx,sy,sz-0.045)
        ua_mx=(sx+ex)/2; ua_my=(sy+ey)/2; ua_mz=(sz+ez)/2; ua_p=-sp
        place(self.ids['ua_link'],   ua_mx,ua_my,ua_mz,ua_p,0,arm_rz)
        place(self.ids['ua_stripe'], ua_mx,ua_my,ua_mz,ua_p,0,arm_rz)
        place(self.ids['ua_cover'],  sx,sy,sz+0.065,ua_p,0,arm_rz)
        place(self.ids['j2'],      ex,ey,ez)
        place(self.ids['j2_ring'], ex,ey,ez-0.022)
        fa_mx=(ex+wx2)/2; fa_my=(ey+wy2)/2; fa_mz=(ez+wz2)/2; fa_p=-(sp+ef)
        place(self.ids['fa_link'],   fa_mx,fa_my,fa_mz,fa_p,0,arm_rz)
        place(self.ids['fa_stripe'], fa_mx,fa_my,fa_mz,fa_p,0,arm_rz)
        place(self.ids['fa_cover'],  ex,ey,ez+0.022,fa_p,0,arm_rz)
        place(self.ids['j4'], wx2,wy2,wz2)
        wr_mx=(wx2+hx)/2; wr_my=(wy2+hy)/2; wr_mz=(wz2+hz)/2; wr_p=-(sp+ef+wp)
        place(self.ids['wr_link'], wr_mx,wr_my,wr_mz,wr_p,0,arm_rz)
        place(self.ids['j6'],      hx,hy,hz)
        place(self.ids['flange'],  hx,hy,hz-0.044,wr_p,0,arm_rz)
        place(self.ids['ft'],      hx,hy,hz-0.072,wr_p,0,arm_rz)
        px,py,pz = hx,hy,hz-0.105
        place(self.ids['g_palm'],px,py,pz,wr_p,0,arm_rz)
        gap = lerp(self.FINGER_OPEN, self.FINGER_CLOSED, self.gripper)
        fsx = -gap*math.sin(ay); fsy = gap*math.cos(ay)
        place(self.ids['g_fa_u'],  px+fsx,     py+fsy,     pz-0.115,wr_p,     0,arm_rz)
        place(self.ids['g_fa_l'],  px+fsx*0.6, py+fsy*0.6, pz-0.218,wr_p+0.30,0,arm_rz)
        place(self.ids['g_pad_a'], px+fsx*0.4, py+fsy*0.4, pz-0.262,wr_p,     0,arm_rz)
        place(self.ids['g_tip_a'], px+fsx*0.3, py+fsy*0.3, pz-0.305)
        place(self.ids['g_fb_u'],  px-fsx,     py-fsy,     pz-0.115,wr_p,     0,arm_rz)
        place(self.ids['g_fb_l'],  px-fsx*0.6, py-fsy*0.6, pz-0.218,wr_p-0.30,0,arm_rz)
        place(self.ids['g_pad_b'], px-fsx*0.4, py-fsy*0.4, pz-0.262,wr_p,     0,arm_rz)
        place(self.ids['g_tip_b'], px-fsx*0.3, py-fsy*0.3, pz-0.305)
        place(self.ids['cam'],   hx,hy,hz+0.030,0,0,arm_rz)
        place(self.ids['cam_l'], hx-0.026*math.sin(ay),hy+0.026*math.cos(ay),hz+0.031)
        place(self.ids['cam_r'], hx+0.026*math.sin(ay),hy-0.026*math.cos(ay),hz+0.031)
        place(self.ids['cam_ir'],hx,hy,hz+0.032)

    # ── Manual control step methods ──────────────────────────────────────────
    def drive_forward(self):
        self.x += math.cos(self.yaw) * self.DRIVE_SPEED
        self.y += math.sin(self.yaw) * self.DRIVE_SPEED
        self.wt += 0.18
        self.update()

    def drive_backward(self):
        self.x -= math.cos(self.yaw) * self.DRIVE_SPEED
        self.y -= math.sin(self.yaw) * self.DRIVE_SPEED
        self.wt -= 0.18
        self.update()

    def strafe_left(self):
        self.x += math.cos(self.yaw + math.pi/2) * self.DRIVE_SPEED
        self.y += math.sin(self.yaw + math.pi/2) * self.DRIVE_SPEED
        self.update()

    def strafe_right(self):
        self.x -= math.cos(self.yaw + math.pi/2) * self.DRIVE_SPEED
        self.y -= math.sin(self.yaw + math.pi/2) * self.DRIVE_SPEED
        self.update()

    def turn_left(self):
        self.yaw = norm_angle(self.yaw + self.TURN_SPEED)
        self.update()

    def turn_right(self):
        self.yaw = norm_angle(self.yaw - self.TURN_SPEED)
        self.update()

    def arm_shoulder_up(self):
        self.shoulder_pitch = max(-math.pi/2, self.shoulder_pitch - self.ARM_SPEED)
        self.update()

    def arm_shoulder_down(self):
        self.shoulder_pitch = min(math.pi/2, self.shoulder_pitch + self.ARM_SPEED)
        self.update()

    def arm_yaw_left(self):
        self.arm_yaw = norm_angle(self.arm_yaw + self.ARM_SPEED)
        self.update()

    def arm_yaw_right(self):
        self.arm_yaw = norm_angle(self.arm_yaw - self.ARM_SPEED)
        self.update()

    def arm_elbow_flex(self):
        self.elbow_flex = min(math.pi * 0.9, self.elbow_flex + self.ARM_SPEED)
        self.update()

    def arm_elbow_extend(self):
        self.elbow_flex = max(0.0, self.elbow_flex - self.ARM_SPEED)
        self.update()

    def toggle_gripper(self):
        self.gripper = 0.0 if self.gripper > 0.5 else 1.0
        anim_gripper(self, self.gripper, steps=18)


# ─────────────────────────────────────────────────────────────────────────────
# MOTION HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def _sync(robot, tablet, carry):
    if carry and tablet:
        tablet.move_to(robot.hand_wx, robot.hand_wy,
                       robot.hand_wz - 0.07, robot.yaw)

def walk_to(robot, tx, ty, spd=1.6, tablet=None, carry=False):
    sx, sy = robot.x, robot.y
    d = math.sqrt((tx-sx)**2 + (ty-sy)**2)
    if d < 0.02: return
    turn_to(robot, math.atan2(ty-sy, tx-sx), tablet=tablet, carry=carry)
    steps = max(1, int(d / (spd * TICK)))
    for i in range(steps+1):
        t = ease(i/steps)
        robot.x = lerp(sx, tx, t)
        robot.y = lerp(sy, ty, t)
        robot.wt += 0.18
        robot.update()
        _sync(robot, tablet, carry)
        time.sleep(TICK)
    robot.x, robot.y = tx, ty
    robot.wt = 0.0
    robot.update()
    _sync(robot, tablet, carry)

def turn_to(robot, target_yaw, tablet=None, carry=False):
    start = robot.yaw
    diff  = norm_angle(target_yaw - start)
    if abs(diff) < 0.01: return
    # FIX: steps now scale with angle magnitude — no more jarring fast spins
    steps = max(8, int(abs(diff) / 0.04))
    for i in range(steps+1):
        robot.yaw = start + diff * ease(i/steps)
        robot.update()
        _sync(robot, tablet, carry)
        time.sleep(TICK)

def anim_arm(robot, tsp, tef, twp=0.0, steps=48, tablet=None, carry=False):
    s0,e0,w0 = robot.shoulder_pitch, robot.elbow_flex, robot.wrist_pitch
    for i in range(steps+1):
        t = ease(i/steps)
        robot.shoulder_pitch = lerp(s0,tsp,t)
        robot.elbow_flex     = lerp(e0,tef,t)
        robot.wrist_pitch    = lerp(w0,twp,t)
        robot.update()
        _sync(robot, tablet, carry)
        time.sleep(TICK)

def anim_arm_yaw(robot, target, steps=32, tablet=None, carry=False):
    start = robot.arm_yaw
    diff  = norm_angle(target - start)
    for i in range(steps+1):
        robot.arm_yaw = start + diff * ease(i/steps)
        robot.update()
        _sync(robot, tablet, carry)
        time.sleep(TICK)

def anim_gripper(robot, target, steps=22, tablet=None, carry=False):
    start = robot.gripper
    for i in range(steps+1):
        robot.gripper = lerp(start, target, ease(i/steps))
        robot.update()
        _sync(robot, tablet, carry)
        time.sleep(TICK)


# ─────────────────────────────────────────────────────────────────────────────
# IK
# ─────────────────────────────────────────────────────────────────────────────
def ik_reach(robot, tx, ty, tz):
    sh_x, sh_y, sh_z = robot._w(0, 0, robot.BASE_H + robot.COL_H)
    dx,dy,dz = tx-sh_x, ty-sh_y, tz-sh_z
    ay = math.atan2(dy, dx)
    horiz = math.sqrt(dx*dx + dy*dy)
    dist  = math.sqrt(horiz*horiz + dz*dz)
    L1, L2 = robot.UPPER_LEN, robot.LOWER_LEN + robot.WRIST_LEN
    dist = max(abs(L1-L2)+0.01, min(L1+L2-0.01, dist))
    ef = math.pi - math.acos(max(-1.,(min(1.,(L1*L1+L2*L2-dist*dist)/(2*L1*L2)))))
    sp = math.atan2(dz,horiz) + math.acos(max(-1.,min(1.,(L1*L1+dist*dist-L2*L2)/(2*L1*dist))))
    return ay, sp, ef, -(sp+ef)


# ─────────────────────────────────────────────────────────────────────────────
# STATUS HUD
# FIX: status text placed high above scene (Z = H*2 + 1.2) so it is never
#      occluded by the robot or any scene object
# FIX: using a plain global variable instead of a mutable list wrapper
# ─────────────────────────────────────────────────────────────────────────────
_hud_id = None

def status(msg, col='#1565C0'):
    global _hud_id
    if _hud_id is not None:
        bb.removeDebugObject(_hud_id)
    _hud_id = bb.createDebugText(
        f"  {msg}",
        (-4.0, -ROOM_HD - 0.6, H + 0.5),
        Q0, color=col, size=0.28)
    print(f"   >> {msg}")


# ─────────────────────────────────────────────────────────────────────────────
# CONTROLS HUD  (in-scene text, summarises slider panel)
# ─────────────────────────────────────────────────────────────────────────────
def build_controls_hud():
    # Place controls legend on the open front face (-Y side), stacked downward
    # so it sits visibly in front of the scene at a readable height
    cx   = (PH_X + PT_X) / 2.0   # midpoint between the two room centres
    y    = -ROOM_HD - 0.5         # just in front of the open front edge
    z_base = H * 2 - 0.3          # just below wall top height
    lines = [
        ("[ SLIDER CONTROLS — top-right panel ]", '#FF8F00', 0.22),
        ("Drive X / Drive Y — move robot",         '#E0E0E0', 0.17),
        ("Turn — rotate robot",                    '#E0E0E0', 0.17),
        ("Shoulder / Elbow / Arm Yaw — arm joints",'#E0E0E0', 0.17),
        ("Gripper — 0=open  1=closed",             '#E0E0E0', 0.17),
        ("Run(>0.5) — delivery   Reset(>0.5) — reset", '#69F0AE', 0.17),
    ]
    for i, (txt, col, sz) in enumerate(lines):
        bb.createDebugText(txt, (cx, y, z_base - i * 0.36),
                           Q0, color=col, size=sz)


# ─────────────────────────────────────────────────────────────────────────────
# SLIDER PANEL  — bb.addUserDebugParameter creates draggable sliders that
#                appear in the top-right corner of the browserbotics viewport.
#                bb.readUserDebugParameter reads the current drag value.
#
# DRIVE / TURN sliders:  range -1 → +1, default 0  (spring-back logic in loop)
# ARM sliders:           range matches joint limits, default = rest pose
# GRIPPER slider:        0 = open, 1 = closed
# ACTION sliders:        0 → 1, pulling past 0.5 triggers once then resets
# ─────────────────────────────────────────────────────────────────────────────
def create_sliders():
    # addDebugSlider(name, min, max, default) — creates slider in top-right panel
    # readDebugParameter(name) — reads current value by the same name string
    bb.addDebugSlider("Drive X",   -1,    1,    0)
    bb.addDebugSlider("Drive Y",   -1,    1,    0)
    bb.addDebugSlider("Turn",      -1,    1,    0)
    bb.addDebugSlider("Shoulder",  -1.57, 1.57, -1.2)
    bb.addDebugSlider("Elbow",      0.0,  2.8,   2.1)
    bb.addDebugSlider("Arm Yaw",   -3.14, 3.14,  0.0)
    bb.addDebugSlider("Gripper",    0.0,  1.0,   0.0)
    bb.addDebugSlider("Run",        0.0,  1.0,   0.0)
    bb.addDebugSlider("Reset",      0.0,  1.0,   0.0)
    # Return name strings — readDebugParameter looks up by name
    return {
        'drive_x':  'Drive X',
        'drive_y':  'Drive Y',
        'turn':     'Turn',
        'shoulder': 'Shoulder',
        'elbow':    'Elbow',
        'arm_yaw':  'Arm Yaw',
        'gripper':  'Gripper',
        'run':      'Run',
        'reset':    'Reset',
    }


# ─────────────────────────────────────────────────────────────────────────────
# DELIVERY SEQUENCE  (single run, no loop)
# FIX: counter approach Y offset increased to prevent body/counter clip
# FIX: door waypoints use DOOR_EXIT_X / DOOR_ENTRY_X with correct clearance
# FIX: gripper pick height raised to TABLET_Z + 0.18 so fingers clear counter
# FIX: bedside approach Y offset increased, clears table arm and IV stand
# FIX: inter-run reset shows status + 1.5s pause before starting
# ─────────────────────────────────────────────────────────────────────────────
def run_delivery(robot, tablet, run_no):
    status(f"DELIVERY #{run_no}  ▶  Starting ...", '#FF8F00')
    time.sleep(0.8)

    # Step 1 — navigate to dispensing counter (safe approach distance)
    status("Step 1  |  Moving to dispensing counter", '#1565C0')
    walk_to(robot, TABLET_X - 0.12, TABLET_Y + COUNTER_APPROACH_Y_OFFSET, spd=1.8)
    time.sleep(0.28)

    # Step 2 — IK + extend arm  (pick height raised to clear counter surface)
    status("Step 2  |  IK — targeting medication", '#FF8F00')
    ay, sp, ef, wp = ik_reach(robot, TABLET_X, TABLET_Y, TABLET_Z + 0.18)
    anim_arm_yaw(robot, ay, steps=30)
    time.sleep(0.10)
    status("Step 2  |  Arm extending", '#FF8F00')
    anim_arm(robot, sp, ef, wp, steps=55)
    time.sleep(0.28)

    # Step 3 — pick
    status("Step 3  |  GRIPPER CLOSE  ▶  Medication secured", '#2E7D32')
    anim_gripper(robot, 1.0, steps=24)
    time.sleep(0.40)

    # Step 4 — lift
    status("Step 4  |  Lifting arm for travel", '#1565C0')
    ay2, sp2, ef2, wp2 = ik_reach(robot, TABLET_X, TABLET_Y, TABLET_Z + 0.55)
    anim_arm(robot, sp2, ef2, wp2, steps=32, tablet=tablet, carry=True)
    time.sleep(0.20)

    # Step 5 — approach door (pharmacy side, body clear of wall)
    status("Step 5  |  Moving to door — pharmacy side", '#0288D1')
    walk_to(robot, DOOR_EXIT_X, 0.0, spd=1.7, tablet=tablet, carry=True)
    time.sleep(0.15)

    # Step 6 — cross door (slow, Y=0 centreline through gap)
    status("Step 6  |  PASSING THROUGH DOOR", '#FF6F00')
    walk_to(robot, DOOR_ENTRY_X, 0.0, spd=1.0, tablet=tablet, carry=True)
    time.sleep(0.18)

    # Step 7 — navigate to bedside (safe approach distance)
    status("Step 7  |  Inside patient room — navigating to bedside", '#1565C0')
    walk_to(robot, BEDSIDE_X - 0.12, BEDSIDE_Y + BEDSIDE_APPROACH_Y_OFFSET,
            spd=1.4, tablet=tablet, carry=True)
    time.sleep(0.28)

    # Step 8 — IK aim at bedside table
    status("Step 8  |  IK — targeting bedside table", '#FF8F00')
    ay3, sp3, ef3, wp3 = ik_reach(robot, BEDSIDE_X, BEDSIDE_Y, BEDSIDE_Z + 0.12)
    anim_arm_yaw(robot, ay3, steps=30, tablet=tablet, carry=True)
    time.sleep(0.14)
    status("Step 8  |  Lowering medication onto table", '#FF8F00')
    anim_arm(robot, sp3, ef3, wp3, steps=48, tablet=tablet, carry=True)
    time.sleep(0.34)

    # Step 9 — place
    status("Step 9  |  GRIPPER OPEN  ▶  Medication placed  |  RFID confirmed", '#2E7D32')
    tablet.rest_on_bedside()
    anim_gripper(robot, 0.0, steps=24)
    time.sleep(0.44)

    # Step 10 — retract arm
    status("Step 10  |  Retracting arm", '#607D8B')
    anim_arm(robot, -1.2, 2.1, 0.0, steps=44)
    anim_arm_yaw(robot, 0.0, steps=22)
    time.sleep(0.28)

    # Step 11 — return to door (patient side)
    status("Step 11  |  Returning to door — patient side", '#0288D1')
    walk_to(robot, DOOR_ENTRY_X, 0.0, spd=1.7)
    time.sleep(0.12)

    # Step 12 — cross back through door (slow)
    status("Step 12  |  PASSING BACK THROUGH DOOR", '#FF6F00')
    walk_to(robot, DOOR_EXIT_X, 0.0, spd=1.0)
    time.sleep(0.14)

    # Step 13 — return to dock
    status("Step 13  |  Returning to charging dock", '#607D8B')
    walk_to(robot, DOCK_X, DOCK_Y, spd=1.9)
    turn_to(robot, 0.0)
    robot.update()

    status(f"DELIVERY #{run_no} COMPLETE  ▶  Drag RunDelivery slider to run again  |  Reset slider to reset",
           '#2E7D32')


# ─────────────────────────────────────────────────────────────────────────────
# RESET — move tablet back to counter, robot back to dock
# ─────────────────────────────────────────────────────────────────────────────
def reset_scene(robot, tablet):
    status("RESET  ▶  Returning robot to dock and tablet to counter ...", '#FF8F00')
    time.sleep(0.5)

    # Snap robot back to dock
    robot.x = DOCK_X; robot.y = DOCK_Y; robot.z = 0.0
    robot.yaw = 0.0
    robot.arm_yaw = 0.0
    robot.shoulder_pitch = -1.2
    robot.elbow_flex = 2.1
    robot.wrist_pitch = 0.0
    robot.gripper = 0.0
    robot.wt = 0.0
    robot.update()

    # Snap tablet back to counter
    tablet.rest_on_counter()

    status("RESET COMPLETE  ▶  Ready.  Drag RunDelivery slider to start a delivery.",
           '#2E7D32')
    time.sleep(1.0)


# ─────────────────────────────────────────────────────────────────────────────
# MAIN CONTROL LOOP  — driven entirely by addUserDebugParameter sliders
#
# Drive X / Drive Y / Turn sliders:
#   Dead-zone ±0.15 so the robot stays still when slider is near centre.
#   The sliders do NOT spring back by themselves (PyBullet/browserbotics
#   sliders hold their last position), so the user drags to move then drags
#   back to 0 to stop — exactly like a joystick held in position.
#
# Arm sliders:
#   Directly map slider value → joint angle each frame.  Move a slider and
#   the arm follows in real-time.
#
# Gripper slider:
#   < 0.5 = open,  ≥ 0.5 = closed.  Animates when state changes.
#
# RunDelivery / Reset sliders:
#   Drag past 0.5 to fire once.  Must drag back below 0.5 before it can
#   fire again (one-shot latch), preventing repeated triggers.
# ─────────────────────────────────────────────────────────────────────────────
def main_loop(robot, tablet, sliders):
    run_no       = 0
    delivering   = False
    run_armed    = True   # True = slider is below threshold, ready to fire
    reset_armed  = True
    gripper_state = 0     # 0 = open, 1 = closed

    DEAD = 0.15   # dead-zone for drive sliders

    status("Ready  |  Use sliders (top-right) to drive / run delivery / reset",
           '#FF8F00')

    while True:
        # ── Read all sliders ─────────────────────────────────────────────────
        dx      = bb.readDebugParameter(sliders['drive_x'])
        dy      = bb.readDebugParameter(sliders['drive_y'])
        turn    = bb.readDebugParameter(sliders['turn'])
        sh      = bb.readDebugParameter(sliders['shoulder'])
        el      = bb.readDebugParameter(sliders['elbow'])
        ay      = bb.readDebugParameter(sliders['arm_yaw'])
        gr      = bb.readDebugParameter(sliders['gripper'])
        run_val = bb.readDebugParameter(sliders['run'])
        rst_val = bb.readDebugParameter(sliders['reset'])

        # ── Action: Reset ────────────────────────────────────────────────────
        if rst_val > 0.5 and reset_armed and not delivering:
            reset_armed = False
            reset_scene(robot, tablet)
            # re-sync arm sliders to rest defaults after reset
            gripper_state = 0
        elif rst_val <= 0.5:
            reset_armed = True

        # ── Action: Run Delivery ─────────────────────────────────────────────
        if run_val > 0.5 and run_armed and not delivering:
            run_armed = False
            delivering = True
            run_no += 1
            status(f"DELIVERY #{run_no} queued — resetting positions ...", '#FF8F00')
            time.sleep(1.5)
            robot.x = DOCK_X; robot.y = DOCK_Y
            robot.yaw = 0.0
            robot.arm_yaw = 0.0
            robot.shoulder_pitch = -1.2
            robot.elbow_flex = 2.1
            robot.wrist_pitch = 0.0
            robot.gripper = 0.0
            robot.wt = 0.0
            robot.update()
            tablet.rest_on_counter()
            time.sleep(0.5)
            run_delivery(robot, tablet, run_no)
            delivering = False
        elif run_val <= 0.5:
            run_armed = True

        # ── Manual controls (only when not mid-delivery) ─────────────────────
        if not delivering:
            # Remap slider value so dead-zone edges snap cleanly to 0
            # and full deflection (±1) maps to full speed.
            # Formula: sign(v) * max(0, (|v|-DEAD)/(1-DEAD))
            def remap(v):
                if abs(v) <= DEAD:
                    return 0.0
                return math.copysign((abs(v) - DEAD) / (1.0 - DEAD), v)

            rdx  = remap(dx)
            rdy  = remap(dy)
            rtrn = remap(turn)

            # Drive — forward/back  (full deflection = 0.18 units/frame = ~9 m/s)
            if rdx != 0.0:
                spd = rdx * 0.18
                robot.x += math.cos(robot.yaw) * spd
                robot.y += math.sin(robot.yaw) * spd
                robot.wt += 0.18 * (1 if rdx > 0 else -1)

            # Drive — strafe
            if rdy != 0.0:
                spd = rdy * 0.18
                robot.x += math.cos(robot.yaw + math.pi/2) * spd
                robot.y += math.sin(robot.yaw + math.pi/2) * spd

            # Turn  (full deflection = 0.10 rad/frame)
            if rtrn != 0.0:
                robot.yaw = norm_angle(robot.yaw + rtrn * 0.10)

            # Arm joints — direct from sliders
            robot.shoulder_pitch = sh
            robot.elbow_flex     = el
            robot.arm_yaw        = ay

            # Gripper — animate on state change
            new_gripper_state = 1 if gr >= 0.5 else 0
            if new_gripper_state != gripper_state:
                gripper_state = new_gripper_state
                anim_gripper(robot, float(gripper_state), steps=18)

            robot.update()

        time.sleep(TICK)


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────
print("=" * 68)
print("  TWO-ROOM MEDICAL ROBOT  v4  — full bug-fix + manual controls")
print("=" * 68)

bb.resetSimulation()
bb.addGroundPlane()
Q0 = bb.getQuaternionFromEuler([0, 0, 0])

build_pharmacy()
build_patient_room()
build_shared_wall()

robot  = MedicalRobot()
robot.spawn()
tablet = Tablet()

# FIX: title HUD lines spaced 0.45 apart (was 0.33 / 0.19 — too tight)
# FIX: placed at clearly separate Z levels so they never overlap
cx_title = (PH_X + PT_X) / 2.0
y_title  = -ROOM_HD - 0.5
bb.createDebugText(
    "MEDICAL ROBOT v4  —  Pharmacy to Patient Room",
    (cx_title, y_title, H * 2 + 0.5),
    Q0, color='#1565C0', size=0.28)
bb.createDebugText(
    "[ PHARMACY ]   DOOR ENTRANCE   [ PATIENT ROOM ]",
    (cx_title, y_title, H * 2 + 0.12),
    Q0, color='#90CAF9', size=0.20)

build_controls_hud()

sliders = create_sliders()

# ── AUTO-RUN: play one delivery animation on startup, then enter manual loop
status("Auto-run startup delivery ...", '#FF8F00')
time.sleep(1.0)
run_delivery(robot, tablet, 0)
time.sleep(1.5)

main_loop(robot, tablet, sliders)
