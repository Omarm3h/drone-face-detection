#!/usr/bin/env python3
"""
Quadcopter Zigzag Search + ROS2 Face Detection + 60s Target Tracking

Behavior:
1) Connect to PX4 (SITL or real).
2) Takeoff to SEARCH_ALT.
3) Fly to Boulevard center (home + 250 m east).
4) Execute a zigzag search pattern over the boulevard area.
5) While zigzagging, listen to ROS2 face detection:
   - /face_detection/detected : std_msgs/Bool
   - /face_detection/position : geometry_msgs/Point (x,y,confidence)
6) When target (yellow jacket) detected → track for 60 seconds.
7) After tracking, RTL.
8) Save a PNG plot of:
   - Drone path (search + track)
   - Estimated target path
   - Boulevard center.
"""

import asyncio
import math
import os
from datetime import datetime

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError


# ===================== CONFIG =====================

SEARCH_ALT = 40.0            # meters (relative altitude)
TRACK_ALT = 40.0             # keep same altitude during tracking
TRACK_DURATION_S = 60.0      # seconds

ZIGZAG_WIDTH_M = 120.0       # total width (east-west)
ZIGZAG_LENGTH_M = 200.0      # total length (north-south)
ZIGZAG_ROWS = 6              # number of sweeps
CRUISE_SPEED_MPS = 5.0       # horizontal speed for search
MAX_CLIMB_RATE_MPS = 2.0     # up/down speed for corrections

BOULEVARD_OFFSET_EAST_M = 250.0  # shift from home to "boulevard center" (east)

OFFBOARD_PRIMING_SETPOINTS = 20
DETECTION_CONFIDENCE_STEPS = 5   # how many cycles of "detected" before trusting

LOG_DIR = "mission_logs"


# ===================== GEO UTILS =====================

def meters_to_deg_lat(m):
    return m / 111320.0

def meters_to_deg_lon(m, lat_deg):
    return m / (111320.0 * math.cos(math.radians(lat_deg)))

def generate_zigzag(center_lat, center_lon, width_m, length_m, rows):
    """
    Generate a lat/lon zigzag pattern centered on center_lat/center_lon.
    Pattern runs along north-south, sweeping east-west.
    """
    wps = []
    half_w = width_m / 2.0
    row_spacing = length_m / max(rows - 1, 1)

    for i in range(rows):
        north_offset = -length_m / 2.0 + i * row_spacing
        dy = meters_to_deg_lat(north_offset)

        # Even rows: west -> east, odd rows: east -> west
        if i % 2 == 0:
            x1 = -half_w
            x2 = +half_w
        else:
            x1 = +half_w
            x2 = -half_w

        dx1 = meters_to_deg_lon(x1, center_lat)
        dx2 = meters_to_deg_lon(x2, center_lat)

        wps.append((center_lat + dy, center_lon + dx1))
        wps.append((center_lat + dy, center_lon + dx2))

    return wps


def latlon_to_xy_m(lat, lon, ref_lat, ref_lon):
    """
    Convert lat/lon to local XY in meters relative to reference point.
    """
    dlat = lat - ref_lat
    dlon = lon - ref_lon
    north = dlat * 111320.0
    east = dlon * (111320.0 * math.cos(math.radians(ref_lat)))
    return east, north


# ===================== ROS2 FACE NODE =====================

class FaceTrackingNode(Node):
    """ROS2 Node that listens to face detection topics."""
    def __init__(self):
        super().__init__('face_tracking_node')
        self.face_detected = False
        self.face_position = None  # geometry_msgs/Point
        self.detection_counter = 0

        self.create_subscription(
            Bool,
            '/face_detection/detected',
            self.detection_callback,
            10
        )

        self.create_subscription(
            Point,
            '/face_detection/position',
            self.position_callback,
            10
        )

    def detection_callback(self, msg: Bool):
        self.face_detected = msg.data
        if msg.data:
            self.detection_counter += 1
        else:
            self.detection_counter = max(0, self.detection_counter - 1)

    def position_callback(self, msg: Point):
        self.face_position = msg


# ===================== MAVSDK HELPERS =====================

async def ensure_offboard(drone: System) -> bool:
    print("[OFFBOARD] Priming setpoints...")
    for _ in range(OFFBOARD_PRIMING_SETPOINTS):
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(0.1)
    try:
        await drone.offboard.start()
        print("[OFFBOARD] ✅ Offboard active")
        return True
    except OffboardError as e:
        print(f"[OFFBOARD] ❌ Failed to start offboard: {e}")
        return False


async def fly_to_gps(drone: System,
                     target_lat: float,
                     target_lon: float,
                     target_alt: float,
                     timeout_s: float = 40.0,
                     position_tolerance_m: float = 1.5,
                     alt_tolerance_m: float = 0.7):
    """
    Simple closed-loop NED velocity controller to fly to a GPS point at a given relative altitude.
    """
    loop = asyncio.get_running_loop()
    t0 = loop.time()

    while loop.time() - t0 < timeout_s:
        pos = await anext(drone.telemetry.position())
        cur_lat = pos.latitude_deg
        cur_lon = pos.longitude_deg
        cur_alt = pos.relative_altitude_m

        # Convert error to N/E in meters
        dlat = target_lat - cur_lat
        dlon = target_lon - cur_lon
        north = dlat * 111320.0
        east = dlon * (111320.0 * math.cos(math.radians(cur_lat)))
        horiz_dist = math.sqrt(north**2 + east**2)

        alt_err = target_alt - cur_alt

        # Horizontal velocity
        if horiz_dist > position_tolerance_m:
            speed = min(CRUISE_SPEED_MPS, horiz_dist / 2.0)
            vn = north / horiz_dist * speed
            ve = east / horiz_dist * speed
        else:
            vn = ve = 0.0

        # Vertical velocity (NED: positive down)
        climb_rate = max(-MAX_CLIMB_RATE_MPS,
                         min(MAX_CLIMB_RATE_MPS, alt_err))
        vd = -climb_rate

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(vn, ve, vd, 0.0)
        )

        if horiz_dist < position_tolerance_m and abs(alt_err) < alt_tolerance_m:
            break

        await asyncio.sleep(0.2)


def estimate_target_latlon(drone_lat, drone_lon, drone_alt, heading_deg,
                           image_x: float, image_y: float) -> (float, float):
    """
    Very rough target estimation:
    - Assume camera looking downwards-ish.
    - image_x in [-1,1] (normalized) shifts the bearing left/right a bit.
    - distance ≈ drone_alt * factor.
    You can tune this later.
    """
    # Forward distance from drone to target on ground
    base_distance = drone_alt * 1.2  # meters
    # Lateral shift based on x (normalized)
    max_side_angle_deg = 25.0
    side_angle = image_x * max_side_angle_deg

    bearing_deg = heading_deg + side_angle
    bearing_rad = math.radians(bearing_deg)

    dx = base_distance * math.cos(bearing_rad)
    dy = base_distance * math.sin(bearing_rad)

    target_lat = drone_lat + meters_to_deg_lat(dy)
    target_lon = drone_lon + meters_to_deg_lon(dx, drone_lat)

    return target_lat, target_lon


# ===================== MAIN MISSION =====================

async def run_tracking_mission():
    # 1) Init ROS2
    rclpy.init()
    ros_node = FaceTrackingNode()

    # 2) Init MAVSDK
    drone = System()
    print("[SYSTEM] Connecting to PX4...")
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[SYSTEM] ✅ Connected to PX4")
            break

    print("[SYSTEM] Waiting for GPS...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("[SYSTEM] ✅ GPS OK & home set")
            break

    home = await anext(drone.telemetry.home())
    HOME_LAT = home.latitude_deg
    HOME_LON = home.longitude_deg
    print(f"[HOME] {HOME_LAT:.6f}, {HOME_LON:.6f}")

    # 3) Compute Boulevard center
    BOULEVARD_LAT = HOME_LAT
    BOULEVARD_LON = HOME_LON + meters_to_deg_lon(
        BOULEVARD_OFFSET_EAST_M,
        HOME_LAT
    )
    print(f"[BOULEVARD] Center ≈ {BOULEVARD_LAT:.6f}, {BOULEVARD_LON:.6f}")

    # 4) Arm + Takeoff
    print("\n[TAKEOFF] Arming...")
    await drone.action.arm()
    print("[TAKEOFF] Taking off...")
    await drone.action.set_takeoff_altitude(SEARCH_ALT)
    await drone.action.takeoff()
    await asyncio.sleep(10.0)

    pos = await anext(drone.telemetry.position())
    print(f"[TAKEOFF] Current alt ≈ {pos.relative_altitude_m:.1f} m")

    # 5) Offboard
    if not await ensure_offboard(drone):
        print("[ABORT] Offboard failed, shutting down.")
        ros_node.destroy_node()
        rclpy.shutdown()
        return

    # 6) Fly to boulevard center at SEARCH_ALT
    print("\n[NAV] Flying to boulevard center...")
    await fly_to_gps(drone, BOULEVARD_LAT, BOULEVARD_LON, SEARCH_ALT, timeout_s=60.0)
    print("[NAV] Reached boulevard center (within tolerance).")

    # 7) Generate zigzag pattern over boulevard
    zigzag_wps = generate_zigzag(
        BOULEVARD_LAT,
        BOULEVARD_LON,
        width_m=ZIGZAG_WIDTH_M,
        length_m=ZIGZAG_LENGTH_M,
        rows=ZIGZAG_ROWS
    )
    print(f"[SEARCH] Zigzag waypoints generated: {len(zigzag_wps)}")

    # Paths for plotting
    drone_path = []
    target_path = []

    # 8) SEARCH PHASE: Zigzag until confident detection
    detected_confident = False
    print("[SEARCH] Starting zigzag search over boulevard...")

    for idx, (wp_lat, wp_lon) in enumerate(zigzag_wps):
        print(f"[SEARCH] Leg {idx+1}/{len(zigzag_wps)} → {wp_lat:.6f}, {wp_lon:.6f}")

        # Move toward this WP but keep checking ROS & logging path
        leg_running = True
        leg_start_time = asyncio.get_running_loop().time()

        while leg_running:
            # Process ROS callbacks
            rclpy.spin_once(ros_node, timeout_sec=0.01)

            # Telemetry
            pos = await anext(drone.telemetry.position())
            heading = await anext(drone.telemetry.heading())
            drone_lat = pos.latitude_deg
            drone_lon = pos.longitude_deg
            drone_alt = pos.relative_altitude_m
            drone_path.append((drone_lat, drone_lon))

            # Compute move toward WP
            dlat = wp_lat - drone_lat
            dlon = wp_lon - drone_lon
            north = dlat * 111320.0
            east = dlon * (111320.0 * math.cos(math.radians(drone_lat)))
            horiz_dist = math.sqrt(north**2 + east**2)

            alt_err = SEARCH_ALT - drone_alt
            if horiz_dist < 1.0 and abs(alt_err) < 0.7:
                leg_running = False
                continue

            # Horizontal velocity
            if horiz_dist > 0.5:
                speed = min(CRUISE_SPEED_MPS, horiz_dist / 2.0)
                vn = north / horiz_dist * speed
                ve = east / horiz_dist * speed
            else:
                vn = ve = 0.0

            # Vertical velocity
            climb_rate = max(-MAX_CLIMB_RATE_MPS,
                             min(MAX_CLIMB_RATE_MPS, alt_err))
            vd = -climb_rate

            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(vn, ve, vd, 0.0)
            )

            # Check detection
            if ros_node.face_detected and ros_node.face_position is not None:
                if ros_node.detection_counter >= DETECTION_CONFIDENCE_STEPS:
                    print("\n🎯 [DETECTED] Yellow-jacket target detected with confidence.")
                    detected_confident = True
                    leg_running = False
                    break

            # Failsafe: don't stay on one leg forever
            if asyncio.get_running_loop().time() - leg_start_time > 60.0:
                leg_running = False

            await asyncio.sleep(0.2)

        if detected_confident:
            break

    # 9) TRACK PHASE: follow target for TRACK_DURATION_S
    if detected_confident:
        print(f"\n[TRACK] Following target for {TRACK_DURATION_S:.0f} seconds...")

        t0 = asyncio.get_running_loop().time()
        while asyncio.get_running_loop().time() - t0 < TRACK_DURATION_S:
            rclpy.spin_once(ros_node, timeout_sec=0.01)

            pos = await anext(drone.telemetry.position())
            heading = await anext(drone.telemetry.heading())
            drone_lat = pos.latitude_deg
            drone_lon = pos.longitude_deg
            drone_alt = pos.relative_altitude_m
            drone_path.append((drone_lat, drone_lon))

            # If still detecting, update estimated target location
            if ros_node.face_detected and ros_node.face_position is not None:
                img_x = ros_node.face_position.x   # assuming normalized [-1,1]
                img_y = ros_node.face_position.y   # unused here, but available
                tgt_lat, tgt_lon = estimate_target_latlon(
                    drone_lat,
                    drone_lon,
                    drone_alt,
                    heading.heading_deg,
                    img_x,
                    img_y
                )
                target_path.append((tgt_lat, tgt_lon))

                # Move toward target
                dlat = tgt_lat - drone_lat
                dlon = tgt_lon - drone_lon
                north = dlat * 111320.0
                east = dlon * (111320.0 * math.cos(math.radians(drone_lat)))
                horiz_dist = math.sqrt(north**2 + east**2)

                alt_err = TRACK_ALT - drone_alt

                if horiz_dist > 0.5:
                    speed = min(CRUISE_SPEED_MPS, horiz_dist / 2.0)
                    vn = north / horiz_dist * speed
                    ve = east / horiz_dist * speed
                else:
                    vn = ve = 0.0

                climb_rate = max(-MAX_CLIMB_RATE_MPS,
                                 min(MAX_CLIMB_RATE_MPS, alt_err))
                vd = -climb_rate

                await drone.offboard.set_velocity_ned(
                    VelocityNedYaw(vn, ve, vd, 0.0)
                )
            else:
                # If target temporarily lost, just loiter (no move)
                await drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )

            await asyncio.sleep(0.2)
    else:
        print("\n[TRACK] No confident detection during search. Skipping tracking phase.")

    # 10) RTL
    print("\n[RTL] Returning to launch...")
    try:
        await drone.offboard.stop()
    except Exception:
        pass
    await drone.action.return_to_launch()
    await asyncio.sleep(20.0)

    # 11) Plot mission paths
    os.makedirs(LOG_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    png_path = os.path.join(LOG_DIR, f"zigzag_tracking_{ts}.png")

    if drone_path:
        drone_x, drone_y = zip(*[latlon_to_xy_m(lat, lon, BOULEVARD_LAT, BOULEVARD_LON)
                                 for (lat, lon) in drone_path])
    else:
        drone_x, drone_y = [], []

    if target_path:
        tgt_x, tgt_y = zip(*[latlon_to_xy_m(lat, lon, BOULEVARD_LAT, BOULEVARD_LON)
                             for (lat, lon) in target_path])
    else:
        tgt_x, tgt_y = [], []

    plt.figure(figsize=(8, 8))
    if drone_x:
        plt.plot(drone_x, drone_y, label="Drone path", linewidth=1.5)
    if target_path:
        plt.plot(tgt_x, tgt_y, 'r--', label="Target path", linewidth=1.5)

    # Boulevard center
    plt.scatter([0.0], [0.0], c='green', marker='x', s=80, label="Boulevard center")

    plt.xlabel("East [m]")
    plt.ylabel("North [m]")
    plt.title("Search and Tracking Paths")
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(png_path, dpi=150)
    plt.close()

    print(f"[PLOT] Mission path plot saved at: {png_path}")

    # 12) Clean ROS2
    ros_node.destroy_node()
    rclpy.shutdown()


def main():
    try:
        asyncio.run(run_tracking_mission())
    except KeyboardInterrupt:
        print("\n[ABORT] Mission interrupted by user.")
    except Exception as e:
        print(f"\n[ERROR] Fatal exception: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
