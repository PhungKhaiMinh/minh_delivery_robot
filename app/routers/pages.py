"""
Router cho các trang giao diện (HTML pages).
Render template Jinja2 và truyền dữ liệu cần thiết.
"""

from fastapi import APIRouter, Request
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.templating import Jinja2Templates
from datetime import datetime

from app.config import CAMPUS_LOCATIONS
from app.services.auth_service import get_current_user
from app.services.booking_service import get_user_bookings, get_active_bookings, get_booking_by_id
from app.services.db_service import db

templates = Jinja2Templates(directory="app/templates")

router = APIRouter(tags=["Pages"])


def _get_robot_info() -> dict:
    """Lấy thông tin robot mặc định."""
    robot = db.collection("robots").document("robot_01").get()
    if robot:
        status_map = {"idle": "Rảnh", "busy": "Đang giao", "charging": "Đang sạc"}
        return {
            "battery": robot.get("battery", 0),
            "status": status_map.get(robot.get("status", "idle"), robot.get("status")),
            "location": "Thư viện Trung tâm",
        }
    return {"battery": 87, "status": "Rảnh", "location": "Thư viện Trung tâm"}


@router.get("/", response_class=HTMLResponse)
async def page_home(request: Request):
    """Trang chủ — redirect đến dashboard nếu đã đăng nhập."""
    user = get_current_user(request)
    if user:
        return RedirectResponse(url="/dashboard", status_code=302)
    return RedirectResponse(url="/login", status_code=302)


@router.get("/login", response_class=HTMLResponse)
async def page_login(request: Request):
    user = get_current_user(request)
    if user:
        return RedirectResponse(url="/dashboard", status_code=302)
    return templates.TemplateResponse("auth/login.html", {"request": request})


@router.get("/register", response_class=HTMLResponse)
async def page_register(request: Request):
    user = get_current_user(request)
    if user:
        return RedirectResponse(url="/dashboard", status_code=302)
    return templates.TemplateResponse("auth/register.html", {"request": request})


@router.get("/dashboard", response_class=HTMLResponse)
async def page_dashboard(request: Request):
    user = get_current_user(request)
    if not user:
        return RedirectResponse(url="/login", status_code=302)

    bookings = get_user_bookings(user.id)

    # Tính thống kê đơn hàng
    stats = {"pending": 0, "in_progress": 0, "completed": 0, "total": len(bookings)}
    for b in bookings:
        s = b.get("status", "")
        if s in ("pending", "confirmed"):
            stats["pending"] += 1
        elif s == "in_progress":
            stats["in_progress"] += 1
        elif s == "completed":
            stats["completed"] += 1

    current_date = datetime.now().strftime("%A, %d/%m/%Y")

    return templates.TemplateResponse("dashboard.html", {
        "request": request,
        "user": user,
        "active_page": "dashboard",
        "stats": stats,
        "recent_bookings": bookings[:5],
        "current_date": current_date,
    })


@router.get("/booking", response_class=HTMLResponse)
async def page_booking(request: Request):
    user = get_current_user(request)
    if not user:
        return RedirectResponse(url="/login", status_code=302)

    today = datetime.now().strftime("%Y-%m-%d")

    return templates.TemplateResponse("booking.html", {
        "request": request,
        "user": user,
        "active_page": "booking",
        "locations": CAMPUS_LOCATIONS,
        "today": today,
        "current_date": datetime.now().strftime("%A, %d/%m/%Y"),
    })


@router.get("/order/{booking_id}", response_class=HTMLResponse)
async def page_order_detail(booking_id: str, request: Request):
    """Chi tiết đơn hàng — tích hợp bản đồ theo dõi robot nếu đơn đang active."""
    user = get_current_user(request)
    if not user:
        return RedirectResponse(url="/login", status_code=302)

    booking = get_booking_by_id(booking_id)
    if not booking or booking.get("user_id") != user.id:
        return RedirectResponse(url="/history", status_code=302)

    active_statuses = {"pending", "confirmed", "in_progress"}
    is_active = booking.get("status") in active_statuses
    robot = _get_robot_info()

    return templates.TemplateResponse("order_detail.html", {
        "request": request,
        "user": user,
        "active_page": "history",
        "booking": booking,
        "is_active": is_active,
        "robot_status": robot["status"],
        "robot_battery": robot["battery"],
        "locations": CAMPUS_LOCATIONS,
        "current_date": datetime.now().strftime("%A, %d/%m/%Y"),
    })


@router.get("/history", response_class=HTMLResponse)
async def page_history(request: Request):
    user = get_current_user(request)
    if not user:
        return RedirectResponse(url="/login", status_code=302)

    bookings = get_user_bookings(user.id)

    return templates.TemplateResponse("history.html", {
        "request": request,
        "user": user,
        "active_page": "history",
        "bookings": bookings,
        "current_date": datetime.now().strftime("%A, %d/%m/%Y"),
    })


@router.get("/profile", response_class=HTMLResponse)
async def page_profile(request: Request):
    user = get_current_user(request)
    if not user:
        return RedirectResponse(url="/login", status_code=302)

    return templates.TemplateResponse("profile.html", {
        "request": request,
        "user": user,
        "active_page": "profile",
        "current_date": datetime.now().strftime("%A, %d/%m/%Y"),
    })
