#!/usr/bin/env python3
"""Extract an Egorov OS EGORAW1 log from a disk image or read-only Windows PhysicalDrive.

The raw writer intentionally overwrites LBA 0 and the next few sectors. Use this
helper only to read/extract the resulting payload. It never writes to the source.
"""
import argparse
import ctypes
import os
import struct
import sys
import zlib
from pathlib import Path

SECTOR = 512
MAGIC = b"EGORAW1\0"
VERSION = 1
HEADER_SIZE = 80

class Source:
    def __init__(self, path: str):
        self.path = path
        self.handle = None
        self.file = None
        self.winapi = None
        if path.startswith(r"\\.\PhysicalDrive"):
            if os.name != "nt":
                raise RuntimeError("\"\\\\.\\PhysicalDriveN\" доступен только в Windows.")
            self._open_physical_read_only(path)
        else:
            self.file = open(path, "rb")

    def _open_physical_read_only(self, path: str):
        from ctypes import wintypes
        k32 = ctypes.WinDLL("kernel32", use_last_error=True)
        k32.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
                                   wintypes.LPVOID, wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
        k32.CreateFileW.restype = wintypes.HANDLE
        k32.SetFilePointerEx.argtypes = [wintypes.HANDLE, ctypes.c_longlong,
                                        ctypes.POINTER(ctypes.c_longlong), wintypes.DWORD]
        k32.SetFilePointerEx.restype = wintypes.BOOL
        k32.ReadFile.argtypes = [wintypes.HANDLE, wintypes.LPVOID, wintypes.DWORD,
                                ctypes.POINTER(wintypes.DWORD), wintypes.LPVOID]
        k32.ReadFile.restype = wintypes.BOOL
        k32.CloseHandle.argtypes = [wintypes.HANDLE]
        k32.CloseHandle.restype = wintypes.BOOL
        # GENERIC_READ only: the physical source is opened read-only.
        handle = k32.CreateFileW(path, 0x80000000, 0x1 | 0x2, None, 3, 0x80, None)
        if handle == wintypes.HANDLE(-1).value:
            raise OSError(ctypes.get_last_error(), "Не удалось открыть физический диск. Запустите терминал от имени администратора и проверьте номер диска.")
        self.handle = handle
        self.winapi = k32

    def read_at(self, offset: int, size: int) -> bytes:
        if offset < 0 or size < 0:
            raise ValueError("Отрицательное смещение/размер")
        if self.file is not None:
            self.file.seek(offset)
            data = self.file.read(size)
            if len(data) != size:
                raise EOFError(f"Короткое чтение: нужно {size}, получено {len(data)} байт")
            return data
        from ctypes import wintypes
        new_pos = ctypes.c_longlong()
        if not self.winapi.SetFilePointerEx(self.handle, ctypes.c_longlong(offset), ctypes.byref(new_pos), 0):
            raise OSError(ctypes.get_last_error(), "Не удалось переместить указатель чтения")
        buf = ctypes.create_string_buffer(size)
        got = wintypes.DWORD(0)
        if not self.winapi.ReadFile(self.handle, buf, size, ctypes.byref(got), None):
            raise OSError(ctypes.get_last_error(), "Не удалось прочитать физический диск")
        if got.value != size:
            raise EOFError(f"Короткое чтение: нужно {size}, получено {got.value} байт")
        return buf.raw

    def close(self):
        if self.file is not None:
            self.file.close()
            self.file = None
        if self.handle is not None:
            self.winapi.CloseHandle(self.handle)
            self.handle = None


def u32(buf: bytes, off: int) -> int:
    return struct.unpack_from("<I", buf, off)[0]


def main() -> int:
    ap = argparse.ArgumentParser(description="Read and extract an Egorov OS raw journal without modifying the source disk.")
    ap.add_argument("--source", required=True, help=r"Disk image file or read-only Windows device, e.g. \\.\PhysicalDrive3")
    ap.add_argument("--output", default="EGOROV.LOG", help="output text log file (default: EGOROV.LOG)")
    ap.add_argument("--force", action="store_true", help="allow overwriting the output file if it already exists")
    args = ap.parse_args()
    out_path = Path(args.output)
    if out_path.exists() and not args.force:
        print(f"ERROR: output already exists: {out_path}; use --force to overwrite it", file=sys.stderr)
        return 2

    src = None
    try:
        src = Source(args.source)
        header = src.read_at(0, SECTOR)
        if header[:8] != MAGIC:
            raise ValueError("EGORAW1 signature not found at LBA 0; wrong drive/image or RAW writing did not finish")
        version = u32(header, 8)
        header_size = u32(header, 12)
        payload_len = u32(header, 16)
        payload_crc = u32(header, 20)
        total_sectors = u32(header, 24)
        header_crc = u32(header, 28)
        if version != VERSION or header_size != HEADER_SIZE:
            raise ValueError(f"Unsupported raw format: version={version}, header_size={header_size}")
        if zlib.crc32(header[:28]) & 0xFFFFFFFF != header_crc:
            raise ValueError("Header CRC32 mismatch")
        needed = 1 + (payload_len + SECTOR - 1) // SECTOR
        if total_sectors != needed or needed < 2:
            raise ValueError("Header size/sectors are inconsistent")
        payload = src.read_at(SECTOR, payload_len)
        if zlib.crc32(payload) & 0xFFFFFFFF != payload_crc:
            raise ValueError("Payload CRC32 mismatch; log may be incomplete or corrupted")
        with open(out_path, "xb" if not args.force else "wb") as f:
            f.write(payload)
        print(f"OK: extracted {payload_len} bytes to {out_path}; CRC32 verified")
        print("Source opened read-only; no sectors were written by this tool.")
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if src is not None:
            src.close()

if __name__ == "__main__":
    raise SystemExit(main())
