"""
API Router cho quản lý thông tin cá nhân (Profile).
"""

from fastapi import APIRouter, Request, Form
from fastapi.responses import JSONResponse

from app.services.auth_service import get_current_user
from app.services.db_service import db
from app.models.user import UserUpdate

router = APIRouter(prefix="/api/profile", tags=["Profile"])


@router.get("/")
async def api_get_profile(request: Request):
    """Lấy thông tin profile hiện tại."""
    user = get_current_user(request)
    if not user:
        return JSONResponse(status_code=401, content={"success": False, "message": "Vui lòng đăng nhập"})

    return JSONResponse(content={
        "success": True,
        "user": {
            "id": user.id,
            "name": user.name,
            "email": user.email,
            "phone": user.phone,
            "created_at": user.created_at,
        },
    })


@router.post("/update")
async def api_update_profile(
    request: Request,
    name: str = Form(None),
    phone: str = Form(None),
):
    """Cập nhật thông tin cá nhân."""
    user = get_current_user(request)
    if not user:
        return JSONResponse(status_code=401, content={"success": False, "message": "Vui lòng đăng nhập"})

    try:
        update_data = UserUpdate(name=name, phone=phone)
        fields_to_update = {}

        if update_data.name:
            fields_to_update["name"] = update_data.name
        if update_data.phone:
            fields_to_update["phone"] = update_data.phone

        if not fields_to_update:
            return JSONResponse(status_code=400, content={"success": False, "message": "Không có thông tin nào để cập nhật"})

        success = db.collection("users").document(user.id).update(fields_to_update)
        if not success:
            return JSONResponse(status_code=500, content={"success": False, "message": "Lỗi cập nhật"})

        return JSONResponse(content={
            "success": True,
            "message": "Cập nhật thông tin thành công",
        })

    except ValueError as e:
        return JSONResponse(status_code=400, content={"success": False, "message": str(e)})
    except Exception as e:
        print(f"[PROFILE ROUTER] Lỗi cập nhật profile: {e}")
        return JSONResponse(status_code=500, content={"success": False, "message": "Lỗi hệ thống"})
