"""
Data models cho User (Sinh viên).
Sử dụng Pydantic để validation input nghiêm ngặt.
"""

from pydantic import BaseModel, EmailStr, field_validator
from typing import Optional
from app.config import ALLOWED_EMAIL_DOMAIN


class UserRegister(BaseModel):
    """Schema đăng ký tài khoản mới."""
    name: str
    phone: str
    email: EmailStr
    password: str

    @field_validator("name")
    @classmethod
    def validate_name(cls, v: str) -> str:
        v = v.strip()
        if len(v) < 2:
            raise ValueError("Họ tên phải có ít nhất 2 ký tự")
        if len(v) > 100:
            raise ValueError("Họ tên không được quá 100 ký tự")
        return v

    @field_validator("phone")
    @classmethod
    def validate_phone(cls, v: str) -> str:
        v = v.strip().replace(" ", "").replace("-", "")
        if not v.startswith(("0", "+84")):
            raise ValueError("Số điện thoại phải bắt đầu bằng 0 hoặc +84")
        digits = v.replace("+", "")
        if not digits.isdigit() or len(digits) < 9 or len(digits) > 12:
            raise ValueError("Số điện thoại không hợp lệ (9-12 chữ số)")
        return v

    @field_validator("email")
    @classmethod
    def validate_hcmut_email(cls, v: str) -> str:
        if not v.lower().endswith(ALLOWED_EMAIL_DOMAIN):
            raise ValueError(f"Email phải có đuôi {ALLOWED_EMAIL_DOMAIN}")
        return v.lower()

    @field_validator("password")
    @classmethod
    def validate_password(cls, v: str) -> str:
        if len(v) < 6:
            raise ValueError("Mật khẩu phải có ít nhất 6 ký tự")
        if len(v) > 128:
            raise ValueError("Mật khẩu không được quá 128 ký tự")
        return v


class UserLogin(BaseModel):
    """Schema đăng nhập."""
    email: EmailStr
    password: str


class UserProfile(BaseModel):
    """Schema hiển thị thông tin người dùng (không bao gồm password)."""
    id: str
    name: str
    phone: str
    email: str
    created_at: Optional[str] = None


class UserUpdate(BaseModel):
    """Schema cập nhật profile."""
    name: Optional[str] = None
    phone: Optional[str] = None

    @field_validator("name")
    @classmethod
    def validate_name(cls, v: Optional[str]) -> Optional[str]:
        if v is not None:
            v = v.strip()
            if len(v) < 2:
                raise ValueError("Họ tên phải có ít nhất 2 ký tự")
        return v

    @field_validator("phone")
    @classmethod
    def validate_phone(cls, v: Optional[str]) -> Optional[str]:
        if v is not None:
            v = v.strip().replace(" ", "").replace("-", "")
            if not v.startswith(("0", "+84")):
                raise ValueError("Số điện thoại phải bắt đầu bằng 0 hoặc +84")
        return v
