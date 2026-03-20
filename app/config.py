"""
Cấu hình chung cho toàn bộ hệ thống BKBookBot.
Quản lý các biến môi trường, đường dẫn, và thiết lập bảo mật.
"""

import os
from pathlib import Path

# === Đường dẫn gốc của dự án ===
BASE_DIR = Path(__file__).resolve().parent.parent

# === Cấu hình lưu trữ dữ liệu offline (mô phỏng Firebase) ===
DATA_DIR = BASE_DIR / "data" / "collections"
USERS_COLLECTION = DATA_DIR / "users"
BOOKINGS_COLLECTION = DATA_DIR / "bookings"
ROBOTS_COLLECTION = DATA_DIR / "robots"

# === Cấu hình bảo mật JWT ===
SECRET_KEY = os.getenv("SECRET_KEY", "bkbookbot-secret-key-change-in-production-2026")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 60 * 24  # Token hết hạn sau 24 giờ

# === Cấu hình server điều khiển Robot ===
ROBOT_SERVER_URL = os.getenv("ROBOT_SERVER_URL", "http://localhost:5001")

# === Cấu hình email sinh viên HCMUT ===
ALLOWED_EMAIL_DOMAIN = "@hcmut.edu.vn"

# === Các địa điểm trong khuôn viên trường cho drop-down ===
CAMPUS_LOCATIONS = [
    {"id": "lib_main", "name": "Thư viện Trung tâm", "lat": 10.7724, "lng": 106.6580},
    {"id": "lib_cs", "name": "Thư viện Khoa CNTT", "lat": 10.7730, "lng": 106.6590},
    {"id": "hall_a", "name": "Hội trường A (Sảnh chính)", "lat": 10.7718, "lng": 106.6575},
    {"id": "hall_b", "name": "Tòa B (Cổng sau)", "lat": 10.7715, "lng": 106.6585},
    {"id": "canteen", "name": "Căn tin Khu A", "lat": 10.7720, "lng": 106.6570},
    {"id": "dorm", "name": "Ký túc xá ĐHBK", "lat": 10.7735, "lng": 106.6595},
    {"id": "gate_ly_thuong_kiet", "name": "Cổng Lý Thường Kiệt", "lat": 10.7710, "lng": 106.6565},
    {"id": "gate_to_hien_thanh", "name": "Cổng Tô Hiến Thành", "lat": 10.7740, "lng": 106.6600},
]

# === Cấu hình ứng dụng ===
APP_NAME = "BK BookBot"
APP_DESCRIPTION = "Hệ thống quản lý Robot giao sách tự hành - ĐHBK TP.HCM"
APP_VERSION = "1.0.0"
