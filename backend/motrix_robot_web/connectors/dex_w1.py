from __future__ import annotations

from typing import Any

from ..safety import SafetyError
from ..schemas import ObservationSnapshot, RobotDescriptor
from .unavailable import UnavailableConnector


class DexW1ReadOnlyConnector(UnavailableConnector):
    """Strictly read-only SSE mirror consumer for legacy PC2:8090 dexforce_web_control.

    By design (per kuawei_roadmap_review & PDF roadmap), Dexforce W1 stays frozen on PC2;
    this unified platform consumes its read-only SSE stream for 3D/Motrix visualization & diagnostics
    and rejects all reverse control writes.
    """

    def __init__(self, descriptor: RobotDescriptor, base_url: str) -> None:
        self.descriptor = descriptor.model_copy(update={"readonly_only": True, "integration_status": "unimplemented", "connector_type": "dex_w1"})
        self.base_url = base_url.rstrip("/")

    def contract_mapping(self) -> dict[str, str]:
        return {
            "sse_mirror": f"GET {self.base_url}/api/events (strictly read-only)",
            "motrix_state": f"GET {self.base_url}/api/motrix/state",
            "write_policy": "FORBIDDEN (403) — W1 legacy write path stays exclusively on PC2",
        }
