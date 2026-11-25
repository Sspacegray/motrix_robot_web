from __future__ import annotations

from typing import Any

from ..safety import SafetyError
from ..schemas import ObservationSnapshot, RobotDescriptor
from .unavailable import UnavailableConnector


class R1ProBridgeConnector(UnavailableConnector):
    """Transition bridge to existing pc10:8080 r1pro_web_server (192.168.3.x isolated NAT).

    Preserves existing R1 Pro perception, actions, workflows, and navigation without rewriting ROS 2;
    enforces single-entry anti-dual-write policy during migration to Edge.
    """

    def __init__(self, descriptor: RobotDescriptor, base_url: str) -> None:
        self.descriptor = descriptor.model_copy(update={"readonly_only": True, "integration_status": "unimplemented", "connector_type": "r1pro"})
        self.base_url = base_url.rstrip("/")

    def contract_mapping(self) -> dict[str, str]:
        return {
            "state": f"GET {self.base_url}/api/state",
            "events": f"GET {self.base_url}/api/events",
            "camera_mjpeg": f"GET {self.base_url}/stream/{{key}}",
            "joint_targets": f"POST {self.base_url}/api/control/joint_targets",
            "navigation": f"POST {self.base_url}/api/navigation/{{mode|goal|cancel|emergency_stop}}",
            "vision": f"POST {self.base_url}/api/vision/* & /api/foundation_pose/* & /api/aruco/*",
            "vla": f"POST {self.base_url}/api/vla/{{connect|load_task|infer|execute|mcap_eval_*}}",
        }
