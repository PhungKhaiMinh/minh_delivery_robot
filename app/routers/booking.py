"""
API Router cho quản lý đơn đặt lịch giao sách.
"""

from fastapi import APIRouter, Request, Form
from fastapi.responses import JSONResponse

from app.models.booking import BookingCreate
from app.services.auth_service import get_current_user
from app.services.booking_service import (
    create_booking,
    get_user_bookings,
    get_booking_by_id,
    cancel_booking,
    get_active_bookings,
)

router = APIRouter(prefix="/api/bookings", tags=["Bookings"])


@router.post("/create")
async def api_create_booking(
    request: Request,
    pickup_location_id: str = Form(...),
    pickup_date: str = Form(...),
    pickup_time: str = Form(...),
    book_count: int = Form(...),
    note: str = Form(""),
):
    """Tạo đơn đặt lịch giao sách mới."""
    user = get_current_user(request)
    if not user:
        return JSONResponse(status_code=401, content={"success": False, "message": "Vui lòng đăng nhập"})

    try:
        data = BookingCreate(
            pickup_location_id=pickup_location_id,
            pickup_date=pickup_date,
            pickup_time=pickup_time,
            book_count=book_count,
            note=note,
        )

        booking, error = await create_booking(user.id, data)
        if error:
            return JSONResponse(status_code=400, content={"success": False, "message": error})

        return JSONResponse(content={
            "success": True,
            "message": "Đã tạo đơn đặt lịch thành công",
            "booking": {
                "id": booking.id,
                "status": booking.status,
                "pickup_location": booking.pickup_location_name,
                "pickup_date": booking.pickup_date,
                "pickup_time": booking.pickup_time,
                "book_count": booking.book_count,
            },
        })

    except ValueError as e:
        return JSONResponse(status_code=400, content={"success": False, "message": str(e)})
    except Exception as e:
        print(f"[BOOKING ROUTER] Lỗi tạo booking: {e}")
        return JSONResponse(status_code=500, content={"success": False, "message": "Lỗi hệ thống"})


@router.get("/my-orders")
async def api_my_orders(request: Request):
    """Lấy danh sách đơn hàng của user hiện tại."""
    user = get_current_user(request)
    if not user:
        return JSONResponse(status_code=401, content={"success": False, "message": "Vui lòng đăng nhập"})

    bookings = get_user_bookings(user.id)

    return JSONResponse(content={
        "success": True,
        "bookings": [
            {
                "id": b.get("_id"),
                "pickup_location_name": b.get("pickup_location_name"),
                "pickup_date": b.get("pickup_date"),
                "pickup_time": b.get("pickup_time"),
                "book_count": b.get("book_count"),
                "status": b.get("status"),
                "note": b.get("note", ""),
                "created_at": b.get("_created_at"),
                "robot_id": b.get("robot_id"),
            }
            for b in bookings
        ],
    })


@router.get("/active")
async def api_active_orders(request: Request):
    """Lấy các đơn đang hoạt động (để theo dõi real-time)."""
    user = get_current_user(request)
    if not user:
        return JSONResponse(status_code=401, content={"success": False, "message": "Vui lòng đăng nhập"})

    bookings = get_active_bookings(user.id)

    return JSONResponse(content={
        "success": True,
        "bookings": [
            {
                "id": b.get("_id"),
                "pickup_location_name": b.get("pickup_location_name"),
                "status": b.get("status"),
                "robot_id": b.get("robot_id"),
                "eta_minutes": b.get("eta_minutes"),
            }
            for b in bookings
        ],
    })


@router.get("/{booking_id}")
async def api_get_booking(booking_id: str, request: Request):
    """Lấy chi tiết một đơn hàng."""
    user = get_current_user(request)
    if not user:
        return JSONResponse(status_code=401, content={"success": False, "message": "Vui lòng đăng nhập"})

    booking = get_booking_by_id(booking_id)
    if not booking:
        return JSONResponse(status_code=404, content={"success": False, "message": "Không tìm thấy đơn hàng"})

    if booking.get("user_id") != user.id:
        return JSONResponse(status_code=403, content={"success": False, "message": "Không có quyền truy cập"})

    return JSONResponse(content={
        "success": True,
        "booking": {
            "id": booking.get("_id"),
            "pickup_location_name": booking.get("pickup_location_name"),
            "pickup_location_id": booking.get("pickup_location_id"),
            "pickup_date": booking.get("pickup_date"),
            "pickup_time": booking.get("pickup_time"),
            "book_count": booking.get("book_count"),
            "status": booking.get("status"),
            "note": booking.get("note", ""),
            "robot_id": booking.get("robot_id"),
            "eta_minutes": booking.get("eta_minutes"),
            "created_at": booking.get("_created_at"),
        },
    })


@router.post("/{booking_id}/cancel")
async def api_cancel_booking(booking_id: str, request: Request):
    """Hủy đơn hàng."""
    user = get_current_user(request)
    if not user:
        return JSONResponse(status_code=401, content={"success": False, "message": "Vui lòng đăng nhập"})

    success, message = cancel_booking(booking_id, user.id)

    if not success:
        return JSONResponse(status_code=400, content={"success": False, "message": message})

    return JSONResponse(content={"success": True, "message": message})
