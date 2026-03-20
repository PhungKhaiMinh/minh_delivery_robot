"""
Data models cho Booking (Đơn đặt lịch giao sách).
"""

from pydantic import BaseModel, field_validator
from typing import Optional
from enum import Enum


class BookingStatus(str, Enum):
    """Các trạng thái của đơn hàng."""
    PENDING = "pending"           # Đang chờ robot xác nhận
    CONFIRMED = "confirmed"       # Robot đã xác nhận, đang trên đường đến
    IN_PROGRESS = "in_progress"   # Robot đang vận chuyển sách
    COMPLETED = "completed"       # Giao sách thành công
    CANCELLED = "cancelled"       # Đơn bị hủy


class BookingCreate(BaseModel):
    """Schema tạo đơn đặt lịch mới."""
    pickup_location_id: str
    pickup_date: str
    pickup_time: str
    book_count: int
    note: Optional[str] = ""

    @field_validator("book_count")
    @classmethod
    def validate_book_count(cls, v: int) -> int:
        if v < 1:
            raise ValueError("Số lượng sách phải ít nhất là 1")
        if v > 20:
            raise ValueError("Tối đa 20 cuốn sách mỗi lần giao")
        return v

    @field_validator("pickup_date")
    @classmethod
    def validate_date(cls, v: str) -> str:
        v = v.strip()
        if not v:
            raise ValueError("Vui lòng chọn ngày nhận sách")
        return v

    @field_validator("pickup_time")
    @classmethod
    def validate_time(cls, v: str) -> str:
        v = v.strip()
        if not v:
            raise ValueError("Vui lòng chọn giờ nhận sách")
        return v


class BookingResponse(BaseModel):
    """Schema phản hồi thông tin đơn hàng."""
    id: str
    user_id: str
    pickup_location_id: str
    pickup_location_name: str
    pickup_date: str
    pickup_time: str
    book_count: int
    note: str
    status: BookingStatus
    robot_id: Optional[str] = None
    eta_minutes: Optional[int] = None
    created_at: Optional[str] = None
    updated_at: Optional[str] = None
