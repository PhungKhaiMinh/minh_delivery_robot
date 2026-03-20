"""
Data models cho Robot giao sách tự hành.
"""

from pydantic import BaseModel
from typing import Optional
from enum import Enum


class RobotStatus(str, Enum):
    """Các trạng thái hoạt động của Robot."""
    IDLE = "idle"                 # Rảnh, sẵn sàng nhận đơn
    BUSY = "busy"                 # Đang thực hiện giao sách
    CHARGING = "charging"         # Đang sạc pin
    MAINTENANCE = "maintenance"   # Đang bảo trì
    OFFLINE = "offline"           # Mất kết nối


class RobotPosition(BaseModel):
    """Vị trí hiện tại của robot (dùng cho real-time tracking)."""
    robot_id: str
    lat: float
    lng: float
    heading: Optional[float] = 0.0  # Hướng di chuyển (0-360 độ)
    speed: Optional[float] = 0.0    # Tốc độ hiện tại (m/s)
    battery: Optional[int] = 100    # Phần trăm pin
    timestamp: Optional[str] = None


class RobotInfo(BaseModel):
    """Thông tin đầy đủ của robot."""
    id: str
    name: str
    status: RobotStatus
    battery: int
    current_lat: Optional[float] = None
    current_lng: Optional[float] = None
    current_task_id: Optional[str] = None
    last_seen: Optional[str] = None
