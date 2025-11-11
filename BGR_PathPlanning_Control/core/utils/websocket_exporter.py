# core/websocket_exporter.py
# ---------------------------------------------------------------
# A tiny, thread-safe broadcast WebSocket server for telemetry.
# ---------------------------------------------------------------
from __future__ import annotations

import asyncio
import functools
import json
import logging
import threading
from typing import Any, Dict, Set

import numpy as np
import websockets                   # pip install websockets>=9

log = logging.getLogger("WebSocketExporter")


class WebSocketExporter:
    """Broadcast JSON frames to every connected WebSocket client."""

    def __init__(self, port: int = 8765):
        self.port: int = port
        self.loop: asyncio.AbstractEventLoop | None = None
        self.queue: asyncio.Queue | None = None
        self.clients: Set[websockets.WebSocketServerProtocol] = set()
        self.ready = threading.Event()                # set when loop + queue ready

    # ─────────────────────────────────────────────────────────────
    # Public API
    # ─────────────────────────────────────────────────────────────
    def run_in_thread(self) -> None:
        """Start server in a daemon thread with its own event-loop."""
        def _thread_target() -> None:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.queue = asyncio.Queue()
            self.ready.set()                          # now safe to .send()
            self.loop.create_task(self._start_server())
            self.loop.run_forever()

        threading.Thread(
            target=_thread_target,
            daemon=True,
            name="WebSocketExporterThread",
        ).start()
        log.info("WebSocket thread spawned on port %d", self.port)

    def send(self, frame: Dict[str, Any]) -> None:
        """Thread-safe enqueue; drops frame if server not ready yet."""
        if not self.ready.is_set() or not self.loop or not self.queue:
            log.debug("WS exporter not ready – frame dropped")
            return

        def _enqueue() -> None:
            try:
                self.queue.put_nowait(frame)
            except asyncio.QueueFull:
                log.warning("WS queue full – frame dropped")

        self.loop.call_soon_threadsafe(_enqueue)

    # ─────────────────────────────────────────────────────────────
    # Internal asyncio logic
    # ─────────────────────────────────────────────────────────────
    async def _start_server(self) -> None:
        async with websockets.serve(
            functools.partial(self._handler),          # binds self
            "0.0.0.0",
            self.port,
        ):
            log.info("WebSocket listening on ws://0.0.0.0:%d", self.port)
            await self._broadcast_loop()

    async def _handler(self, websocket): 
        self.clients.add(websocket)
        log.info("Client connected")
        try:
            await websocket.wait_closed()
        finally:
            self.clients.discard(websocket)
            log.info("Client disconnected")

    async def _broadcast_loop(self) -> None:
        assert self.queue is not None
        while True:
            frame = await self.queue.get()
            try:
                payload = json.dumps(frame, default=self._json_default)
            except (TypeError, ValueError) as e:
                log.error("JSON serialisation failed – skipped: %s", e)
                continue

            dead = set()
            for ws in self.clients:
                try:
                    await ws.send(payload)
                except Exception as e:
                    log.warning("Send failed, dropping client: %s", e)
                    dead.add(ws)
            self.clients.difference_update(dead)

    # ─────────────────────────────────────────────────────────────
    # Fallback JSON encoder
    # ─────────────────────────────────────────────────────────────
    @staticmethod
    def _json_default(obj):
        if isinstance(obj, (np.integer, np.floating)):
            return obj.item()
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if hasattr(obj, "__dict__"):
            return obj.__dict__
        return str(obj)
