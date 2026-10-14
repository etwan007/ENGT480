"""Compile and run TX1 fault-injection tests with the available host compiler."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class TxDiagnosticsTests(unittest.TestCase):
    def test_transmitter_failure_paths_and_callback_versions(self):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if compiler is None:
            self.skipTest("A host C++ compiler is required")
        tests = Path(__file__).resolve().parent
        with tempfile.TemporaryDirectory(prefix="tx1-diagnostics-") as directory:
            for minor in (3, 5):
                with self.subTest(idf_minor=minor):
                    binary = str(Path(directory) / f"tx1-idf5{minor}")
                    subprocess.run([
                        compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror",
                        f"-DMOCK_IDF_MINOR={minor}", "-I", str(tests / "tx_mocks"),
                        str(tests / "tx_diagnostics_test.cpp"), "-o", binary,
                    ], check=True, capture_output=True, text=True)
                    completed = subprocess.run([binary], check=True, capture_output=True, text=True)
                    self.assertIn("TX1 diagnostics checks passed", completed.stdout)


if __name__ == "__main__":
    unittest.main()
