# Array of detected obstacles — primary input for obstacle avoidance.
#
# Published at ~10 Hz on /obstacles topic.
# QoS: BestEffort + KeepLast(1) — always latest data, never stale.
#
# Usage in subscriber:
#   from delivery_interfaces.msg import ObstacleArray
#   self.create_subscription(ObstacleArray, '/obstacles', cb, qos_profile)

std_msgs/Header header      # stamp + frame_id (laser frame)

Obstacle[] obstacles         # Detected obstacles sorted by distance (nearest first)

uint32 seq                   # Sequence counter — gaps indicate dropped frames
bool lidar_active            # True if LiDAR is currently providing /scan data
float32 compute_ms           # Processing time for this cycle (ms) for load monitoring
