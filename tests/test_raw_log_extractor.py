#!/usr/bin/env python3
"""Host-side round-trip test for EGORAW1 extraction; uses a synthetic image only."""
import subprocess
import sys
import tempfile
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXTRACTOR = ROOT / "tools" / "extract_egorov_raw_log.py"
SECTOR = 512
payload = (b"Egorov OS test log\r\n" + bytes(range(256)) + b"\x00END\n") * 7
sectors = 1 + (len(payload) + SECTOR - 1) // SECTOR
header = bytearray(SECTOR)
header[0:8] = b"EGORAW1\x00"
header[8:12] = (1).to_bytes(4, "little")
header[12:16] = (80).to_bytes(4, "little")
header[16:20] = len(payload).to_bytes(4, "little")
header[20:24] = (zlib.crc32(payload) & 0xFFFFFFFF).to_bytes(4, "little")
header[24:28] = sectors.to_bytes(4, "little")
header[28:32] = (zlib.crc32(header[:28]) & 0xFFFFFFFF).to_bytes(4, "little")
header[32:32 + len(b"Egorov OS raw journal; payload at LBA 1")] = b"Egorov OS raw journal; payload at LBA 1"
with tempfile.TemporaryDirectory() as td:
    td = Path(td)
    image = td / "synthetic.img"
    output = td / "EGOROV.LOG"
    padded = payload + b"\x00" * ((SECTOR - len(payload) % SECTOR) % SECTOR)
    image.write_bytes(bytes(header) + padded)
    run = subprocess.run([sys.executable, str(EXTRACTOR), "--source", str(image), "--output", str(output)],
                         capture_output=True, text=True)
    if run.returncode != 0:
        print(run.stdout, end="")
        print(run.stderr, end="", file=sys.stderr)
        raise SystemExit("extractor failed on valid synthetic image")
    if output.read_bytes() != payload:
        raise SystemExit("extracted bytes do not match payload")
    # A corrupted image must fail verification and must not emit a successful log.
    bad_image = td / "bad.img"
    damaged = bytearray(image.read_bytes())
    damaged[SECTOR + 3] ^= 0x40
    bad_image.write_bytes(damaged)
    bad_output = td / "BAD.LOG"
    bad = subprocess.run([sys.executable, str(EXTRACTOR), "--source", str(bad_image), "--output", str(bad_output)],
                         capture_output=True, text=True)
    if bad.returncode == 0 or bad_output.exists():
        raise SystemExit("extractor accepted corrupted payload")
print("PASS: EGORAW1 extraction round-trip and corruption detection")
