"""
Entry point chính của ứng dụng BK BookBot.
Khởi tạo FastAPI app, đăng ký routers, và cấu hình middleware.
"""

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

from app.config import APP_NAME, APP_DESCRIPTION, APP_VERSION
from app.routers import auth, booking, tracking, profile, pages
from app.services.robot_service import init_default_robot


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Khởi tạo dữ liệu khi server bắt đầu chạy."""
    print(f"\n{'='*50}")
    print(f"  {APP_NAME} v{APP_VERSION}")
    print(f"  {APP_DESCRIPTION}")
    print(f"{'='*50}\n")

    # Tạo robot mặc định cho demo
    init_default_robot()
    print("[STARTUP] Hệ thống sẵn sàng!\n")

    yield

    print("\n[SHUTDOWN] Đang tắt hệ thống...")


app = FastAPI(
    title=APP_NAME,
    description=APP_DESCRIPTION,
    version=APP_VERSION,
    lifespan=lifespan,
)

# === CORS middleware (cho phép gọi API từ frontend) ===
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# === Mount static files ===
app.mount("/static", StaticFiles(directory="app/static"), name="static")

# === Đăng ký API routers ===
app.include_router(auth.router)
app.include_router(booking.router)
app.include_router(tracking.router)
app.include_router(profile.router)

# === Đăng ký Page router (phải sau API routers) ===
app.include_router(pages.router)
