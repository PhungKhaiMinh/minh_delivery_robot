"""
API Router cho xác thực người dùng.
Xử lý đăng ký, đăng nhập, đăng xuất.
"""

from fastapi import APIRouter, Request, Form, Response
from fastapi.responses import RedirectResponse, JSONResponse

from app.models.user import UserRegister, UserLogin
from app.services.auth_service import register_user, login_user, get_current_user

router = APIRouter(prefix="/api/auth", tags=["Authentication"])


@router.post("/register")
async def api_register(
    name: str = Form(...),
    phone: str = Form(...),
    email: str = Form(...),
    password: str = Form(...),
    confirm_password: str = Form(...),
):
    """API đăng ký tài khoản mới."""
    try:
        if password != confirm_password:
            return JSONResponse(
                status_code=400,
                content={"success": False, "message": "Mật khẩu xác nhận không khớp"},
            )

        data = UserRegister(name=name, phone=phone, email=email, password=password)
        profile, error = register_user(data)

        if error:
            return JSONResponse(
                status_code=400,
                content={"success": False, "message": error},
            )

        return JSONResponse(
            content={"success": True, "message": "Đăng ký thành công! Vui lòng đăng nhập."},
        )

    except ValueError as e:
        error_msg = str(e)
        # Pydantic validation trả về danh sách lỗi
        if "validation error" in error_msg.lower():
            error_msg = "Thông tin không hợp lệ. Vui lòng kiểm tra lại."
        return JSONResponse(
            status_code=400,
            content={"success": False, "message": error_msg},
        )
    except Exception as e:
        print(f"[AUTH ROUTER] Lỗi đăng ký: {e}")
        return JSONResponse(
            status_code=500,
            content={"success": False, "message": "Lỗi hệ thống"},
        )


@router.post("/login")
async def api_login(
    email: str = Form(...),
    password: str = Form(...),
):
    """API đăng nhập."""
    try:
        token, profile, error = login_user(email, password)

        if error:
            return JSONResponse(
                status_code=401,
                content={"success": False, "message": error},
            )

        response = JSONResponse(
            content={
                "success": True,
                "message": "Đăng nhập thành công",
                "user": {
                    "id": profile.id,
                    "name": profile.name,
                    "email": profile.email,
                },
            }
        )

        # Lưu JWT vào cookie HttpOnly (bảo mật, chống XSS)
        response.set_cookie(
            key="access_token",
            value=token,
            httponly=True,
            max_age=60 * 60 * 24,  # 24 giờ
            samesite="lax",
            secure=False,  # True nếu dùng HTTPS
        )

        return response

    except Exception as e:
        print(f"[AUTH ROUTER] Lỗi đăng nhập: {e}")
        return JSONResponse(
            status_code=500,
            content={"success": False, "message": "Lỗi hệ thống"},
        )


@router.post("/logout")
async def api_logout():
    """Đăng xuất — xóa cookie JWT."""
    response = JSONResponse(content={"success": True, "message": "Đã đăng xuất"})
    response.delete_cookie("access_token")
    return response


@router.get("/me")
async def api_get_me(request: Request):
    """Lấy thông tin user hiện tại từ token."""
    user = get_current_user(request)
    if not user:
        return JSONResponse(
            status_code=401,
            content={"success": False, "message": "Chưa đăng nhập"},
        )
    return JSONResponse(
        content={
            "success": True,
            "user": {
                "id": user.id,
                "name": user.name,
                "email": user.email,
                "phone": user.phone,
                "created_at": user.created_at,
            },
        }
    )
