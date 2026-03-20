from __future__ import annotations

"""
Module mô phỏng cấu trúc Firestore (NoSQL) của Firebase.
Dữ liệu được lưu dưới dạng file JSON theo cấu trúc Collections/Documents.

Cấu trúc thư mục:
    data/collections/
    ├── users/
    │   ├── {user_id}.json       <- Mỗi document là 1 file JSON riêng
    │   └── ...
    ├── bookings/
    │   ├── {booking_id}.json
    │   └── ...
    └── robots/
        ├── {robot_id}.json
        └── ...

API được thiết kế giống Firebase Firestore để dễ dàng migration sau này:
    db = FirestoreOffline("data/collections")
    db.collection("users").add({"name": "Minh", ...})
    db.collection("users").document("user_123").get()
    db.collection("users").where("email", "==", "minh@hcmut.edu.vn")
"""

import json
import uuid
import threading
from pathlib import Path
from datetime import datetime, timezone
from typing import Any, Optional


class Document:
    """
    Đại diện cho một Document trong Firestore.
    Mỗi document tương ứng với một file JSON trên đĩa.
    """

    def __init__(self, collection_path: Path, doc_id: str):
        self.collection_path = collection_path
        self.id = doc_id
        self.file_path = collection_path / f"{doc_id}.json"
        self._lock = threading.Lock()

    def get(self) -> Optional[dict]:
        """Đọc dữ liệu document. Trả về None nếu không tồn tại."""
        try:
            if not self.file_path.exists():
                return None
            with open(self.file_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            data["_id"] = self.id
            return data
        except (json.JSONDecodeError, IOError) as e:
            print(f"[DB ERROR] Không thể đọc document {self.id}: {e}")
            return None

    def set(self, data: dict, merge: bool = False) -> bool:
        """
        Ghi dữ liệu vào document.
        Nếu merge=True, chỉ cập nhật các trường được cung cấp (giữ nguyên trường cũ).
        Nếu merge=False, ghi đè toàn bộ document.
        """
        try:
            with self._lock:
                write_data = {}
                if merge and self.file_path.exists():
                    with open(self.file_path, "r", encoding="utf-8") as f:
                        write_data = json.load(f)

                write_data.update(data)
                write_data["_updated_at"] = datetime.now(timezone.utc).isoformat()

                if "_created_at" not in write_data:
                    write_data["_created_at"] = write_data["_updated_at"]

                # Loại bỏ _id khỏi dữ liệu lưu trữ (ID nằm trong tên file)
                write_data.pop("_id", None)

                self.collection_path.mkdir(parents=True, exist_ok=True)
                with open(self.file_path, "w", encoding="utf-8") as f:
                    json.dump(write_data, f, ensure_ascii=False, indent=2, default=str)
                return True
        except (IOError, TypeError) as e:
            print(f"[DB ERROR] Không thể ghi document {self.id}: {e}")
            return False

    def update(self, data: dict) -> bool:
        """Cập nhật một phần document (merge tự động)."""
        if not self.file_path.exists():
            print(f"[DB WARNING] Document {self.id} không tồn tại để update.")
            return False
        return self.set(data, merge=True)

    def delete(self) -> bool:
        """Xóa document khỏi collection."""
        try:
            if self.file_path.exists():
                self.file_path.unlink()
                return True
            return False
        except IOError as e:
            print(f"[DB ERROR] Không thể xóa document {self.id}: {e}")
            return False

    @property
    def exists(self) -> bool:
        return self.file_path.exists()


class Collection:
    """
    Đại diện cho một Collection trong Firestore.
    Mỗi collection tương ứng với một thư mục chứa các file JSON.
    """

    def __init__(self, base_path: Path, name: str):
        self.name = name
        self.path = base_path / name
        self.path.mkdir(parents=True, exist_ok=True)

    def document(self, doc_id: str) -> Document:
        """Truy cập document theo ID."""
        return Document(self.path, doc_id)

    def add(self, data: dict, doc_id: Optional[str] = None) -> tuple[str, bool]:
        """
        Thêm document mới vào collection.
        Tự động tạo ID nếu không cung cấp (giống Firestore auto-ID).
        Trả về (document_id, success).
        """
        if doc_id is None:
            doc_id = self._generate_id()

        doc = Document(self.path, doc_id)
        if doc.exists:
            print(f"[DB WARNING] Document {doc_id} đã tồn tại trong {self.name}.")
            return doc_id, False

        data["_created_at"] = datetime.now(timezone.utc).isoformat()
        success = doc.set(data)
        return doc_id, success

    def get_all(self) -> list[dict]:
        """Lấy tất cả documents trong collection."""
        documents = []
        try:
            for file_path in sorted(self.path.glob("*.json")):
                try:
                    with open(file_path, "r", encoding="utf-8") as f:
                        data = json.load(f)
                    data["_id"] = file_path.stem
                    documents.append(data)
                except (json.JSONDecodeError, IOError):
                    continue
        except IOError as e:
            print(f"[DB ERROR] Không thể đọc collection {self.name}: {e}")
        return documents

    def where(self, field: str, op: str, value: Any) -> list[dict]:
        """
        Truy vấn documents theo điều kiện (mô phỏng Firestore query).
        Hỗ trợ: ==, !=, >, <, >=, <=, in, contains
        """
        results = []
        ops = {
            "==": lambda a, b: a == b,
            "!=": lambda a, b: a != b,
            ">": lambda a, b: a > b,
            "<": lambda a, b: a < b,
            ">=": lambda a, b: a >= b,
            "<=": lambda a, b: a <= b,
            "in": lambda a, b: a in b,
            "contains": lambda a, b: b in a if isinstance(a, (list, str)) else False,
        }

        compare = ops.get(op)
        if compare is None:
            print(f"[DB ERROR] Toán tử '{op}' không được hỗ trợ.")
            return results

        for doc_data in self.get_all():
            try:
                # Hỗ trợ truy vấn nested field (vd: "address.city")
                field_value = self._get_nested_field(doc_data, field)
                if field_value is not None and compare(field_value, value):
                    results.append(doc_data)
            except (TypeError, ValueError):
                continue

        return results

    def count(self) -> int:
        """Đếm số documents trong collection."""
        return len(list(self.path.glob("*.json")))

    def delete_all(self) -> int:
        """Xóa tất cả documents. Trả về số documents đã xóa."""
        deleted = 0
        for file_path in self.path.glob("*.json"):
            try:
                file_path.unlink()
                deleted += 1
            except IOError:
                continue
        return deleted

    def _generate_id(self) -> str:
        """Tạo ID ngẫu nhiên giống Firebase auto-ID (20 ký tự)."""
        return uuid.uuid4().hex[:20]

    @staticmethod
    def _get_nested_field(data: dict, field: str) -> Any:
        """Truy cập trường lồng nhau bằng dấu chấm (vd: 'profile.name')."""
        keys = field.split(".")
        current = data
        for key in keys:
            if isinstance(current, dict) and key in current:
                current = current[key]
            else:
                return None
        return current


class FirestoreOffline:
    """
    Interface chính mô phỏng Firebase Firestore.

    Sử dụng:
        db = FirestoreOffline("/path/to/data/collections")

        # Thêm document
        user_id, ok = db.collection("users").add({"name": "Minh", "email": "minh@hcmut.edu.vn"})

        # Đọc document
        user = db.collection("users").document(user_id).get()

        # Truy vấn
        active_bookings = db.collection("bookings").where("status", "==", "pending")

        # Cập nhật
        db.collection("users").document(user_id).update({"phone": "0901234567"})

        # Xóa
        db.collection("users").document(user_id).delete()
    """

    def __init__(self, base_path: str | Path):
        self.base_path = Path(base_path)
        self.base_path.mkdir(parents=True, exist_ok=True)

    def collection(self, name: str) -> Collection:
        """Truy cập collection theo tên."""
        return Collection(self.base_path, name)


# === Singleton instance cho toàn bộ ứng dụng ===
from app.config import DATA_DIR

db = FirestoreOffline(DATA_DIR)
