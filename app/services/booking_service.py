from __future__ import annotations

"""
Module quản lý đơn đặt lịch giao sách (Booking).
Xử lý tạo, cập nhật, hủy đơn và truy vấn lịch sử.
"""

from typing import Optional
from datetime import datetime, timezone

from app.services.db_service import db
from app.services.robot_service import send_booking_to_robot
from app.models.booking import BookingCreate, BookingStatus, BookingResponse
from app.config import CAMPUS_LOCATIONS


def get_location_by_id(location_id: str) -> Optional[dict]:
    """Tìm thông tin địa điểm theo ID từ danh sách cấu hình."""
    for loc in CAMPUS_LOCATIONS:
        if loc["id"] == location_id:
            return loc
    return None


async def create_booking(user_id: str, data: BookingCreate) -> tuple[Optional[BookingResponse], str]:
    """
    Tạo đơn đặt lịch mới.
    Tự động gửi thông tin đến Robot Server.
    """
    try:
        # Xác thực địa điểm
        location = get_location_by_id(data.pickup_location_id)
        if not location:
            return None, "Địa điểm không hợp lệ"

        # Tạo dữ liệu booking
        booking_data = {
            "user_id": user_id,
            "pickup_location_id": data.pickup_location_id,
            "pickup_location_name": location["name"],
            "pickup_lat": location["lat"],
            "pickup_lng": location["lng"],
            "pickup_date": data.pickup_date,
            "pickup_time": data.pickup_time,
            "book_count": data.book_count,
            "note": data.note or "",
            "status": BookingStatus.PENDING,
            "robot_id": None,
            "eta_minutes": None,
        }

        # Lưu vào database
        booking_id, success = db.collection("bookings").add(booking_data)
        if not success:
            return None, "Lỗi hệ thống khi tạo đơn"

        # Gửi đơn đến Robot Server (bất đồng bộ, không block nếu server offline)
        robot_success, robot_msg = await send_booking_to_robot(booking_id, booking_data)
        if robot_success:
            db.collection("bookings").document(booking_id).update({
                "status": BookingStatus.CONFIRMED,
                "robot_id": "robot_01",
            })
            booking_data["status"] = BookingStatus.CONFIRMED
            booking_data["robot_id"] = "robot_01"

        response = BookingResponse(
            id=booking_id,
            user_id=user_id,
            pickup_location_id=data.pickup_location_id,
            pickup_location_name=location["name"],
            pickup_date=data.pickup_date,
            pickup_time=data.pickup_time,
            book_count=data.book_count,
            note=data.note or "",
            status=booking_data["status"],
            robot_id=booking_data.get("robot_id"),
        )

        return response, ""

    except Exception as e:
        print(f"[BOOKING ERROR] Lỗi tạo booking: {e}")
        return None, "Đã xảy ra lỗi không mong muốn"


def get_user_bookings(user_id: str) -> list[dict]:
    """Lấy tất cả đơn hàng của một user, sắp xếp mới nhất trước."""
    try:
        bookings = db.collection("bookings").where("user_id", "==", user_id)
        bookings.sort(key=lambda x: x.get("_created_at", ""), reverse=True)
        return bookings
    except Exception as e:
        print(f"[BOOKING ERROR] Lỗi truy vấn bookings: {e}")
        return []


def get_booking_by_id(booking_id: str) -> Optional[dict]:
    """Lấy chi tiết một đơn hàng theo ID."""
    try:
        return db.collection("bookings").document(booking_id).get()
    except Exception as e:
        print(f"[BOOKING ERROR] Lỗi đọc booking {booking_id}: {e}")
        return None


def cancel_booking(booking_id: str, user_id: str) -> tuple[bool, str]:
    """
    Hủy đơn hàng. Chỉ cho phép hủy khi đơn đang ở trạng thái pending/confirmed.
    Kiểm tra quyền sở hữu đơn.
    """
    try:
        booking = db.collection("bookings").document(booking_id).get()
        if not booking:
            return False, "Không tìm thấy đơn hàng"

        if booking.get("user_id") != user_id:
            return False, "Bạn không có quyền hủy đơn này"

        current_status = booking.get("status")
        if current_status not in [BookingStatus.PENDING, BookingStatus.CONFIRMED]:
            return False, f"Không thể hủy đơn đang ở trạng thái: {current_status}"

        db.collection("bookings").document(booking_id).update({
            "status": BookingStatus.CANCELLED,
            "cancelled_at": datetime.now(timezone.utc).isoformat(),
        })

        return True, "Đã hủy đơn hàng thành công"

    except Exception as e:
        print(f"[BOOKING ERROR] Lỗi hủy booking: {e}")
        return False, "Đã xảy ra lỗi khi hủy đơn"


def get_active_bookings(user_id: str) -> list[dict]:
    """Lấy các đơn đang hoạt động (pending, confirmed, in_progress)."""
    try:
        all_bookings = db.collection("bookings").where("user_id", "==", user_id)
        active_statuses = {BookingStatus.PENDING, BookingStatus.CONFIRMED, BookingStatus.IN_PROGRESS}
        return [b for b in all_bookings if b.get("status") in active_statuses]
    except Exception as e:
        print(f"[BOOKING ERROR] Lỗi truy vấn active bookings: {e}")
        return []
