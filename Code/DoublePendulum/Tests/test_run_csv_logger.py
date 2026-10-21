# Resolve shared modules from this file, including when launched in Thonny.
import sys
from pathlib import Path

CODE_DIRECTORY = Path(__file__).resolve().parents[2]
if str(CODE_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(CODE_DIRECTORY))

import csv
import tempfile
import unittest

from Global.run_csv_logger import COLUMNS, DOUBLE_COLUMNS, RunCsvLogger


class RunCsvLoggerTests(unittest.TestCase):
    def test_records_all_queued_samples_and_stop(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "nested" / "run.csv"
            logger = RunCsvLogger(path, queue_capacity=1000)
            logger.start()
            logger.event("start", "calibrated", 0.0)
            for cycle in range(300):
                row = [""] * len(COLUMNS)
                row[0] = "sample"
                row[1] = cycle / 100
                row[2] = cycle + 1
                logger.sample(row)
            self.assertIsNone(logger.stop("Ctrl+C"))
            with path.open(newline="", encoding="utf-8") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), 302)
            self.assertEqual(rows[0]["record_type"], "start")
            self.assertEqual([int(row["cycle"]) for row in rows[1:-1]], list(range(1, 301)))
            self.assertEqual(rows[-1]["record_type"], "stop")
            self.assertIn("dropped_samples=0", rows[-1]["message"])

    def test_full_queue_drops_without_blocking(self):
        logger = RunCsvLogger(Path("unused.csv"), queue_capacity=1)
        logger.sample(("sample",))
        logger.sample(("sample",))
        self.assertEqual(logger.dropped_samples, 1)

    def test_double_pendulum_schema(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "double.csv"
            logger = RunCsvLogger(path, columns=DOUBLE_COLUMNS)
            logger.start()
            row = [""] * len(DOUBLE_COLUMNS)
            row[0] = "sample"
            row[DOUBLE_COLUMNS.index("measured_angle_2_deg")] = 3.5
            logger.sample(row)
            self.assertIsNone(logger.stop("Ctrl+C"))
            with path.open(newline="", encoding="utf-8") as file:
                reader = csv.DictReader(file)
                self.assertEqual(tuple(reader.fieldnames), DOUBLE_COLUMNS)
                rows = list(reader)
            self.assertEqual(rows[0]["measured_angle_2_deg"], "3.5")
            self.assertEqual(rows[-1]["record_type"], "stop")


if __name__ == "__main__":
    unittest.main()
