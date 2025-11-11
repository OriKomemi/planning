# recording/parquet_recorder.py
# ---------------------------------------------------------------
# A lightweight Parquet recorder for appending frame logs.
# ---------------------------------------------------------------
import logging
from pathlib import Path
from typing import Any, Dict, List
import threading

import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq

log = logging.getLogger("ParquetRecorder")


class ParquetRecorder:
    """Records dictionaries (frames) to a Parquet file efficiently."""

    def __init__(self, path: str, flush_every: int = 100):
        self.path = Path(path)
        self.flush_every = flush_every
        self._buffer: List[Dict[str, Any]] = []
        self._lock = threading.Lock()

    def add(self, frame: Dict[str, Any]) -> None:
        """Add a frame to the buffer and flush if needed."""
        with self._lock:
            self._buffer.append(frame)
            if len(self._buffer) >= self.flush_every:
                self.flush()

    def flush(self) -> None:
        """Write buffered frames to Parquet."""
        if not self._buffer:
            return
        try:
            df = pd.DataFrame(self._buffer)
            table = pa.Table.from_pandas(df)
            if self.path.exists():
                existing = pq.read_table(self.path)
                combined = pa.concat_tables([existing, table])
                pq.write_table(combined, self.path)
            else:
                pq.write_table(table, self.path)
            log.info("Flushed %d frames to Parquet", len(self._buffer))
        except Exception as e:
            log.error("Failed to write to Parquet: %s", e)
        finally:
            self._buffer.clear()

    def close(self) -> None:
        """Flush any remaining data before shutdown."""
        with self._lock:
            self.flush()
