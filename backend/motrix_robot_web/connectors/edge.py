from __future__ import annotations

from typing import Any

from ..safety import SafetyError
from ..schemas import ObservationSnapshot, RobotDescriptor
from .unavailable import UnavailableConnector


class EdgeConnector(UnavailableConnector):
    """Mainline Connector targeting motrix-edge FastAPI /v1 contract.

    Maps:
      - GET  {base_url}/v1/health & /v1/preview  -> ObservationSnapshot (Observation Layout)
      - POST {base_url}/v1/leases                -> Underlying Edge Lease
      - POST {base_url}/v1/commands              -> robot/execute, robot/estop (Action Layout)
      - GET  {base_url}/v1/captures & /v1/infers -> Policy / MCAP status

    Strict boundary: never owns CAN/hardware drivers; defaults to readonly_only=True until P2 gate sign-off.
    """

    def __init__(self, descriptor: RobotDescriptor, base_url: str) -> None:
        self.descriptor = descriptor.model_copy(update={"readonly_only": True, "integration_status": "unimplemented", "connector_type": "edge"})
        self.base_url = base_url.rstrip("/")

    def contract_mapping(self) -> dict[str, str]:
        return {
            "health": f"GET {self.base_url}/v1/health",
            "preview": f"GET {self.base_url}/v1/preview",
            "leases": f"POST {self.base_url}/v1/leases",
            "commands": f"POST {self.base_url}/v1/commands",
            "estop_bypass": f"POST {self.base_url}/v1/commands (command=robot/estop, lease-free)",
            "webrtc": f"POST {self.base_url}/v1/webrtc/offer",
            "infers": f"GET/POST {self.base_url}/v1/infers",
            "captures": f"GET {self.base_url}/v1/captures",
        }
