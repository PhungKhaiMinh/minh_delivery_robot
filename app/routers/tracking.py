from __future__ import annotations

"""
API Router cho theo dõi robot real-time.
Cung cấp endpoint REST và WebSocket.
"""

from fastapi import APIRouter, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
import asyncio
import json

from app.services.auth_service import get_current_user, decode_access_token
from app.services.robot_service import get_robot_position, get_robot_eta
from app.services.booking_service import get_booking_by_id, get_active_bookings

router = APIRouter(prefix="/api/tracking", tags=["Tracking"])

# Danh sách WebSocket connections đang active (để broadcast vị trí robot)
active_connections: list[WebSocket] = []


@router.get("/robot/{robot_id}/position")
async def api_robot_position(robot_id: str, request: Request):
    """Lấy vị trí hiện tại của robot (REST endpoint — dùng cho polling fallback)."""
    user = get_current_user(request)
    if not user:
        return JSONResponse(status_code=401, content={"success": False, "message": "Vui lòng đăng nhập"})

    position = await get_robot_position(robot_id)
    if not position:
        return JSONResponse(status_code=404, content={"success": False, "message": "Không tìm thấy robot"})

    return JSONResponse(content={"success": True, "position": position})


@router.get("/robot/{robot_id}/eta")
async def api_robot_eta(robot_id: str, booking_id: str, request: Request):
    """Lấy ETA (thời gian dự kiến đến) của robot."""
    user = get_current_user(request)
    if not user:
        return JSONResponse(status_code=401, content={"success": False, "message": "Vui lòng đăng nhập"})

    eta = await get_robot_eta(robot_id, booking_id)

    return JSONResponse(content={
        "success": True,
        "eta_minutes": eta if eta is not None else "N/A",
    })


@router.websocket("/ws/{token}")
async def websocket_tracking(websocket: WebSocket, token: str):
    """
    WebSocket endpoint cho real-time robot tracking.
    Client kết nối với token JWT để xác thực.
    Server gửi vị trí robot mỗi 2 giây.
    """
    # Xác thực token
    payload = decode_access_token(token)
    if not payload:
        await websocket.close(code=4001, reason="Token không hợp lệ")
        return

    user_id = payload.get("sub")
    await websocket.accept()
    active_connections.append(websocket)

    try:
        while True:
            # Lấy các đơn đang active của user
            active = get_active_bookings(user_id)

            for booking in active:
                robot_id = booking.get("robot_id")
                if robot_id:
                    position = await get_robot_position(robot_id)
                    if position:
                        await websocket.send_json({
                            "type": "robot_position",
                            "booking_id": booking.get("_id"),
                            "position": position,
                        })

            # Chờ 2 giây trước khi gửi tiếp (tần suất cập nhật)
            await asyncio.sleep(2)

    except WebSocketDisconnect:
        active_connections.remove(websocket)
    except Exception as e:
        print(f"[WS ERROR] {e}")
        if websocket in active_connections:
            active_connections.remove(websocket)
