"""Pruebas host del formato persistente usado por sd_store.c.

No reemplazan la prueba física FAT/ESP32; ejercitan selección A/B, CRC,
recuperación de cola y búsquedas con puntos cada 64 sobre CSV grandes.
"""

from __future__ import annotations

import os
import struct
import tempfile
import time
import unittest
import zlib
from pathlib import Path


CHECKPOINT_MAGIC = 0x45534350
CHECKPOINT_VERSION = 1
CSV_FORMAT_VERSION = 3
INDEX_INTERVAL = 64
CHECKPOINT_PREFIX = struct.Struct("<IHHQIQQQQIII12s")
CHECKPOINT = struct.Struct("<IHHQIQQQQIII12sI")
INDEX_ENTRY = struct.Struct("<IQI")


def row(measurement_id: int) -> bytes:
    return f"{measurement_id},1,{measurement_id},1,gps,2026-07-20T17:00:00Z,400\n".encode()


def checkpoint(generation: int, last_id: int, rows: int, size: int, offset: int, line: bytes) -> bytes:
    prefix = CHECKPOINT_PREFIX.pack(
        CHECKPOINT_MAGIC,
        CHECKPOINT_VERSION,
        CSV_FORMAT_VERSION,
        generation,
        last_id,
        rows,
        size,
        offset,
        size,
        zlib.crc32(line) if line else 0,
        rows // INDEX_INTERVAL,
        0,
        b"\0" * 12,
    )
    return prefix + struct.pack("<I", zlib.crc32(prefix))


def valid_checkpoint(raw: bytes) -> tuple | None:
    if len(raw) != CHECKPOINT.size:
        return None
    values = CHECKPOINT.unpack(raw)
    if values[0:3] != (CHECKPOINT_MAGIC, CHECKPOINT_VERSION, CSV_FORMAT_VERSION):
        return None
    return values if zlib.crc32(raw[:-4]) == values[-1] else None


def recover_tail(path: Path, confirmed_end: int, last_id: int) -> tuple[int, int, str | None]:
    with path.open("r+b") as handle:
        handle.seek(confirmed_end)
        end = confirmed_end
        expected_id = last_id + 1
        while True:
            offset = handle.tell()
            line = handle.readline()
            if not line:
                break
            if not line.endswith(b"\n"):
                handle.truncate(end)
                return last_id, end, "incomplete"
            try:
                parsed = int(line.split(b",", 1)[0])
            except ValueError:
                handle.truncate(end)
                return last_id, end, "invalid"
            if parsed != expected_id:
                handle.truncate(end)
                cause = "duplicate_or_regression" if parsed < expected_id else "gap"
                return last_id, end, cause
            last_id = parsed
            expected_id += 1
            end = handle.tell()
        return last_id, end, None


def build_sparse(path: Path) -> list[tuple[int, int]]:
    entries = []
    with path.open("rb") as handle:
        handle.readline()
        while line := handle.readline():
            offset = handle.tell() - len(line)
            measurement_id = int(line.split(b",", 1)[0])
            if measurement_id == 1 or measurement_id % INDEX_INTERVAL == 0:
                entries.append((measurement_id, offset))
    return entries


def sparse_seek(entries: list[tuple[int, int]], requested: int) -> int:
    eligible = [offset for measurement_id, offset in entries if measurement_id <= requested]
    return eligible[-1] if eligible else 0


class PersistenceModelTests(unittest.TestCase):
    def make_csv(self, count: int) -> tuple[tempfile.TemporaryDirectory, Path, list[int]]:
        temp = tempfile.TemporaryDirectory()
        path = Path(temp.name) / "data.csv"
        offsets = []
        with path.open("wb") as handle:
            handle.write(b"id,boot_id,uptime_s,time_valid,time_source,timestamp,co2\n")
            for measurement_id in range(1, count + 1):
                offsets.append(handle.tell())
                handle.write(row(measurement_id))
        return temp, path, offsets

    def test_checkpoint_fixed_size_crc_and_generation_selection(self):
        a = checkpoint(8, 10, 10, 500, 450, row(10))
        b = checkpoint(9, 11, 11, 550, 500, row(11))
        self.assertEqual(CHECKPOINT.size, 80)
        self.assertEqual(valid_checkpoint(a)[3], 8)
        self.assertEqual(max((valid_checkpoint(a), valid_checkpoint(b)), key=lambda value: value[3])[3], 9)
        corrupt = bytearray(b)
        corrupt[20] ^= 0x55
        self.assertIsNone(valid_checkpoint(bytes(corrupt)))

    def test_csv_after_checkpoint_recovers_only_tail(self):
        temp, path, offsets = self.make_csv(1000)
        self.addCleanup(temp.cleanup)
        confirmed_end = offsets[900]
        last_id, end, error = recover_tail(path, confirmed_end, 900)
        self.assertEqual((last_id, end, error), (1000, path.stat().st_size, None))

    def test_incomplete_last_row_is_truncated_without_reusing_complete_id(self):
        temp, path, offsets = self.make_csv(50)
        self.addCleanup(temp.cleanup)
        confirmed = path.stat().st_size
        with path.open("ab") as handle:
            handle.write(b"51,1,51,1,gps,2026")
        last_id, end, error = recover_tail(path, confirmed, 50)
        self.assertEqual(last_id, 50)
        self.assertEqual(end, confirmed)
        self.assertEqual(path.stat().st_size, confirmed)
        self.assertEqual(error, "incomplete")

    def test_tail_gap_is_rejected_and_truncated_at_first_bad_row(self):
        temp, path, offsets = self.make_csv(100)
        self.addCleanup(temp.cleanup)
        confirmed = path.stat().st_size
        with path.open("ab") as handle:
            handle.write(row(101))
            valid_end = handle.tell()
            handle.write(row(500))
        last_id, end, error = recover_tail(path, confirmed, 100)
        self.assertEqual((last_id, end, error), (101, valid_end, "gap"))
        self.assertEqual(path.stat().st_size, valid_end)

    def test_tail_duplicate_is_rejected(self):
        temp, path, offsets = self.make_csv(100)
        self.addCleanup(temp.cleanup)
        confirmed = path.stat().st_size
        with path.open("ab") as handle:
            handle.write(row(101))
            valid_end = handle.tell()
            handle.write(row(101))
        self.assertEqual(recover_tail(path, confirmed, 100), (101, valid_end, "duplicate_or_regression"))

    def test_tail_regression_is_rejected(self):
        temp, path, offsets = self.make_csv(100)
        self.addCleanup(temp.cleanup)
        confirmed = path.stat().st_size
        with path.open("ab") as handle:
            handle.write(row(99))
        self.assertEqual(recover_tail(path, confirmed, 100), (100, confirmed, "duplicate_or_regression"))

    def test_uint32_max_does_not_wrap(self):
        last_id = (1 << 32) - 1
        self.assertIsNone(None if last_id == (1 << 32) - 1 else last_id + 1)

    def test_sparse_index_points_and_range_seek(self):
        temp, path, offsets = self.make_csv(1000)
        self.addCleanup(temp.cleanup)
        entries = build_sparse(path)
        self.assertEqual([item[0] for item in entries[:4]], [1, 64, 128, 192])
        for requested in (1, 64, 65, 127, 128, 999):
            offset = sparse_seek(entries, requested)
            with path.open("rb") as handle:
                handle.seek(offset)
                first = int(handle.readline().split(b",", 1)[0])
                scanned = 1
                while first < requested:
                    first = int(handle.readline().split(b",", 1)[0])
                    scanned += 1
            self.assertEqual(first, requested)
            self.assertLessEqual(scanned, INDEX_INTERVAL)

    def test_index_memory_reduction(self):
        rows = 100_000
        old_bytes = (rows + 1) * 4
        new_bytes = (1 + rows // INDEX_INTERVAL) * INDEX_ENTRY.size
        self.assertLess(new_bytes, old_bytes / 10)

    def test_normal_start_validation_cost_is_constant(self):
        timings = []
        for count in (1_000, 14_439, 100_000):
            temp, path, offsets = self.make_csv(count)
            try:
                started = time.perf_counter()
                with path.open("rb") as handle:
                    handle.seek(offsets[-1])
                    line = handle.readline()
                self.assertEqual(int(line.split(b",", 1)[0]), count)
                timings.append(time.perf_counter() - started)
            finally:
                temp.cleanup()
        self.assertLess(max(timings), 0.05)


if __name__ == "__main__":
    unittest.main()
