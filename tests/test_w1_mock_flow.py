import importlib.util
from pathlib import Path


def test_w1_mock_flow_is_reproducible_without_network_or_motion(tmp_path):
    script = Path(__file__).resolve().parents[1] / "scripts/verify_w1_mock_flow.py"
    spec = importlib.util.spec_from_file_location("w1_mock_flow", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    report = module.run_flow(tmp_path / "evidence")
    assert report["result"] == "PASS"
    assert len(report["steps"]) == 11
    assert all(step["result"] == "PASS" for step in report["steps"])
