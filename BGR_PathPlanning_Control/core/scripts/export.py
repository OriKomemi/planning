import logging
import sys
import pathlib
logging.basicConfig(level=logging.INFO)

sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent.parent))
print(sys.path)

from core.utils.parquet_recorder import ParquetRecorder
recorder = ParquetRecorder("test_output.parquet", flush_every=2)

frames = [
    {"timestamp": 0.0, "x": 1.0, "y": 2.0},
    {"timestamp": 0.1, "x": 1.1, "y": 2.1},
    {"timestamp": 0.2, "x": 1.2, "y": 2.2},
]

for frame in frames:
    recorder.add(frame)

recorder.close()  # IMPORTANT: Forces write if < flush_every

import os
assert os.path.exists("test_output.parquet")
print("✅ File created:", os.path.abspath("test_output.parquet"))
