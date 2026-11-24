from __future__ import annotations

from typing import Any

from ..safety import SafetyError
from ..schemas import ObservationSnapshot, RobotDescriptor
from .unavailable import UnavailableConnector


class PiperLeLabConnector(UnavailableConnector):
    """Transition bridge to existing motphys:8095 Piper LeLab + omah:18100 OpenPI policy server.

    CRITICAL ARCHITECTURAL INVARIANT:
      - Piper must have ONLY ONE CAN writer (`Robot Pipeline` under motrix-edge OR legacy LeLab, NEVER BOTH).
      - This connector performs an explicit exclusivity guard against any active EdgeConnector on Piper.
    """

    def __init__(self, descriptor: RobotDescriptor, base_url: str, policy_url: str = "") -> None:
        self.descriptor = descriptor.model_copy(update={"readonly_only": True, "integration_status": "unimplemented", "connector_type": "piper_lelab"})
        self.base_url = base_url.rstrip("/")
        self.policy_url = policy_url.rstrip("/")

    @staticmethod
    def assert_no_dual_can_writer(active_connector_types: list[tuple[str, str, bool]]) -> None:
        """Fail closed if both motrix-edge and piper_lelab are configured with write access for Piper."""
        writable_piper = [
            ctype
            for rtype, ctype, readonly in active_connector_types
            if rtype == "piper_dual" and not readonly and ctype in ("edge", "piper_lelab")
        ]
        if "edge" in writable_piper and "piper_lelab" in writable_piper:
            raise SafetyError(
                409,
                "dual_can_writer_forbidden",
                "红线拦截：检测到 Piper 同时启用了可写的 EdgeConnector 与 PiperLeLabConnector，严禁出现第二 CAN 写者！",
            )

    def contract_mapping(self) -> dict[str, str]:
        return {
            "lelab_ws": f"WS {self.base_url}/ws",
            "policy_health": f"GET {self.policy_url or self.base_url}/healthz",
            "checkpoints": f"GET/POST {self.base_url}/api/checkpoints",
            "offline_eval": f"POST {self.base_url}/api/offline_eval",
            "estop": f"POST {self.base_url}/api/estop",
        }
