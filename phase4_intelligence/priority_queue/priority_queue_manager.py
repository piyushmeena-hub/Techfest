"""
UAV-X Phase 4: Priority Queue Manager
Manages the data delivery queue with strict priority ordering.
Ensures CRITICAL data (survivor detections) always reaches GCS first.
Handles store-and-forward when links are temporarily down.
"""
from __future__ import annotations
import heapq
import time
from typing import List, Optional
from core.models import DataPacket, DataPriority


class PriorityQueueManager:
    """
    Priority queue for data packets destined for GCS.
    Priority order: CRITICAL > HIGH > MEDIUM > LOW
    Within same priority: FIFO (earlier created_at first).
    Supports: retry, expiry, max queue size, stats.
    """

    def __init__(self, max_size: int = 1000, max_retries: int = 5,
                 expiry_ticks: int = 100):
        self._heap: List[DataPacket] = []
        self._retry_count: dict = {}   # packet_id -> int
        self.max_size = max_size
        self.max_retries = max_retries
        self.expiry_ticks = expiry_ticks
        self.dropped_count = 0
        self.delivered_count = 0

    def enqueue(self, packet: DataPacket) -> bool:
        """Add packet to queue. Returns False if queue full."""
        if len(self._heap) >= self.max_size:
            # Drop lowest priority packet if new one is higher priority
            if self._heap and self._heap[-1].priority.value > packet.priority.value:
                self._heap.pop()
                heapq.heapify(self._heap)
                self.dropped_count += 1
            else:
                return False
        heapq.heappush(self._heap, packet)
        self._retry_count[packet.packet_id] = 0
        return True

    def peek(self) -> Optional[DataPacket]:
        """Return highest priority packet without removing."""
        return self._heap[0] if self._heap else None

    def dequeue(self) -> Optional[DataPacket]:
        """Remove and return highest priority packet."""
        if not self._heap:
            return None
        return heapq.heappop(self._heap)

    def requeue(self, packet: DataPacket) -> bool:
        """Re-add a failed packet for retry. Returns False if max retries exceeded."""
        self._retry_count[packet.packet_id] = self._retry_count.get(packet.packet_id, 0) + 1
        if self._retry_count[packet.packet_id] > self.max_retries:
            self.dropped_count += 1
            return False
        heapq.heappush(self._heap, packet)
        return True

    def expire_old(self, current_tick: int):
        """Remove packets older than expiry_ticks (if not CRITICAL)."""
        fresh = []
        for packet in self._heap:
            age = current_tick - packet.created_at
            if packet.priority == DataPriority.CRITICAL or age <= self.expiry_ticks:
                fresh.append(packet)
            else:
                self.dropped_count += 1
        heapq.heapify(fresh)
        self._heap = fresh

    def mark_delivered(self, packet_id: str):
        self.delivered_count += 1
        self._retry_count.pop(packet_id, None)

    @property
    def size(self) -> int:
        return len(self._heap)

    def get_stats(self) -> dict:
        by_priority = {}
        for p in self._heap:
            k = p.priority.name
            by_priority[k] = by_priority.get(k, 0) + 1
        return {
            "queue_size": self.size,
            "delivered": self.delivered_count,
            "dropped": self.dropped_count,
            "by_priority": by_priority,
        }
