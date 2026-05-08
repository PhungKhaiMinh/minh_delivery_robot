# Single detected obstacle for obstacle avoidance planning.
#
# Coordinate convention (laser frame):
#   angle > 0 = left of robot front
#   angle < 0 = right of robot front
#   distance  = horizontal plane distance (m)

# --- Data source constants ---
uint8 SOURCE_DEPTH=0
uint8 SOURCE_LIDAR=1
uint8 SOURCE_FUSED=2

# --- Core avoidance data ---
float32 distance            # Best available nearest-point distance (m)
float32 angle               # Center bearing from robot front (rad)
float32 angle_min           # Right edge bearing (rad, most negative)
float32 angle_max           # Left edge bearing (rad, most positive)
float32 width               # Estimated obstacle width at distance (m)

# --- Per-sensor distances (for algorithm to weigh reliability) ---
float32 depth_distance      # Distance from depth camera (m), -1 if N/A
float32 lidar_distance      # Distance from LiDAR (m), -1 if N/A

# --- Classification ---
uint16 class_id             # COCO class ID (0=person, 2=car, 56=chair ...)
string class_name           # Human-readable name, e.g. "person"
float32 confidence          # YOLO detection confidence [0.0 - 1.0]
uint8 source                # SOURCE_DEPTH / SOURCE_LIDAR / SOURCE_FUSED
