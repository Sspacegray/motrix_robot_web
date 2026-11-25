"""Offline asset/route validation for all 3 robot URDFs, textures, and R1 Pro real SLAM map."""
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

from fastapi.testclient import TestClient
from motrix_robot_web.app import create_app
from motrix_robot_web.connectors.mock import _w1_descriptor, _r1pro_descriptor, _piper_descriptor

ROOT = Path(__file__).resolve().parents[1]
W1_ASSET = ROOT / "frontend/assets/urdf/dexforce_w1_v030"
R1_ASSET = ROOT / "frontend/assets/urdf/galaxea_r1pro"
PIPER_ASSET = ROOT / "frontend/assets/urdf/piper_dual_d405"
R1_MAP_ASSET = ROOT / "frontend/assets/maps/r1pro_20260918"


def test_w1_visual_mesh_materials_and_embedded_images_are_complete():
    manifest = json.loads((W1_ASSET / "visual-manifest.json").read_text())
    refs = {n.attrib["filename"] for n in ET.parse(W1_ASSET / "robot.urdf").findall(".//visual/geometry/mesh")}
    assert refs == {f["path"] for f in manifest["files"]}
    assert len(refs) == 21
    images = 0
    for record in manifest["files"]:
        blob = (W1_ASSET / record["path"]).read_bytes()
        assert hashlib.sha256(blob).hexdigest() == record["sha256"]
        assert blob[:4] == b"glTF"
        version, length = struct.unpack_from("<II", blob, 4)
        assert version == 2 and length == len(blob)
        json_length, chunk_type = struct.unpack_from("<II", blob, 12)
        assert chunk_type == 0x4E4F534A
        gltf = json.loads(blob[20:20 + json_length])
        assert gltf.get("meshes") and gltf.get("materials")
        assert not gltf.get("extensionsRequired")
        assert all("uri" not in entry for entry in gltf.get("buffers", []) + gltf.get("images", []))
        images += len(gltf.get("images", []))
    assert images == 10
    names = {n.attrib["name"] for n in ET.parse(W1_ASSET / "robot.urdf").findall("joint")}
    assert all(j.urdf_joint in names for j in _w1_descriptor().joints)
    assert _w1_descriptor().capabilities.urdf_3d


def test_r1pro_and_piper_dual_real_urdf_meshes_textures_and_joint_bindings():
    # Galaxea R1 Pro: 36 UV-mapped OBJ visual meshes + real output.jpg texture atlas + 20 articulated joints
    r1_manifest = json.loads((R1_ASSET / "visual-manifest.json").read_text())
    r1_tree = ET.parse(R1_ASSET / "robot.urdf")
    r1_visual_meshes = r1_tree.findall(".//visual/geometry/mesh")
    assert len(r1_visual_meshes) == 36
    r1_tex = {n.attrib["filename"] for n in r1_tree.findall(".//texture")}
    assert "meshes_textured/output.jpg" in r1_tex
    r1_all_refs = {n.attrib["filename"] for n in r1_visual_meshes} | r1_tex
    assert r1_all_refs == {f["path"] for f in r1_manifest["files"]}
    assert len(r1_manifest["files"]) == 37
    for record in r1_manifest["files"]:
        blob = (R1_ASSET / record["path"]).read_bytes()
        assert hashlib.sha256(blob).hexdigest() == record["sha256"]
        assert len(blob) == record["bytes"]
        if record["path"].endswith(".obj"):
            assert b"\nvt " in blob or blob.startswith(b"vt ")
    r1_joints = {n.attrib["name"] for n in r1_tree.findall("joint")}
    r1_desc = _r1pro_descriptor()
    assert r1_desc.capabilities.urdf_3d is True
    assert r1_desc.urdf_url == "/assets/urdf/galaxea_r1pro/robot.urdf"
    assert all(j.urdf_joint in r1_joints for j in r1_desc.joints)
    assert all(j.urdf_scale == 0.05 for j in r1_desc.joints if j.is_gripper)
    assert len(r1_tree.findall(".//mimic")) >= 2

    # AgileX Piper Dual + D405: 37 unique assets (77 visual refs) + real identification.png texture
    p_manifest = json.loads((PIPER_ASSET / "visual-manifest.json").read_text())
    p_tree = ET.parse(PIPER_ASSET / "robot.urdf")
    p_visual_meshes = p_tree.findall(".//visual/geometry/mesh")
    assert len(p_visual_meshes) == 77
    p_tex = {n.attrib["filename"] for n in p_tree.findall(".//texture")}
    assert "meshes/d405/identification.png" in p_tex
    p_all_refs = {n.attrib["filename"] for n in p_visual_meshes} | p_tex
    assert p_all_refs == {f["path"] for f in p_manifest["files"]}
    assert len(p_manifest["files"]) == 37
    for record in p_manifest["files"]:
        blob = (PIPER_ASSET / record["path"]).read_bytes()
        assert hashlib.sha256(blob).hexdigest() == record["sha256"]
        assert len(blob) == record["bytes"]
    p_joints = {n.attrib["name"] for n in p_tree.findall("joint")}
    p_desc = _piper_descriptor()
    assert p_desc.capabilities.urdf_3d is True
    assert p_desc.urdf_url == "/assets/urdf/piper_dual_d405/robot.urdf"
    assert all(j.urdf_joint in p_joints for j in p_desc.joints)
    assert all(j.urdf_scale == 0.05 for j in p_desc.joints if j.is_gripper)
    assert len(p_tree.findall(".//mimic")) >= 2


def test_r1pro_real_slam_map_manifest_and_integrity():
    for map_dir, expected_id, expected_pts in [
        (ROOT / "frontend/assets/maps/fast_slam_20260914", "fast_slam_20260914", 981005),
        (R1_MAP_ASSET, "r1pro_20260918", 215046),
    ]:
        manifest = json.loads((map_dir / "map-manifest.json").read_text())
        assert manifest["id"] == expected_id
        assert manifest["point_count"] == expected_pts
        for fname, meta in manifest["files"].items():
            blob = (map_dir / fname).read_bytes()
            assert len(blob) == meta["bytes"]
            assert hashlib.sha256(blob).hexdigest() == meta["sha256"]


def test_offline_model_and_loader_routes_and_fresh_brand(tmp_path):
    app = create_app(data_dir=tmp_path)
    with TestClient(app) as client:
        html = client.get("/")
        assert html.status_code == 200
        assert "LOO1-20261124-cancel-map-v5" in html.text
        assert "motphys-logo.svg?v=" in html.text
        assert '<polygon points="12 2 2 22 22 22"' not in html.text
        assert "no-cache" in html.headers["cache-control"]
        for base_url, asset_dir in [
            ("/assets/urdf/dexforce_w1_v030/", W1_ASSET),
            ("/assets/urdf/galaxea_r1pro/", R1_ASSET),
            ("/assets/urdf/piper_dual_d405/", PIPER_ASSET),
        ]:
            urdf_resp = client.get(base_url + "robot.urdf")
            assert urdf_resp.status_code == 200
            urdf_root = ET.fromstring(urdf_resp.content)
            for node in list(urdf_root.findall(".//visual/geometry/mesh")) + list(urdf_root.findall(".//texture")):
                assert client.get(base_url + node.attrib["filename"]).status_code == 200
            manifest = json.loads((asset_dir / "visual-manifest.json").read_text())
            for entry in manifest["files"][:5]:
                response = client.get(base_url + entry["path"])
                assert response.status_code == 200
                assert len(response.content) == entry["bytes"]
        # Explicitly verify Piper D405 texture route
        assert client.get("/assets/urdf/piper_dual_d405/meshes/d405/identification.png").status_code == 200
        # Verify real R1 Pro SLAM map routes
        assert client.get("/assets/maps/r1pro_20260918/r1pro_map.pcd").status_code == 200
        assert client.get("/assets/maps/r1pro_20260918/r1pro_map.png").status_code == 200
        assert client.get("/assets/maps/r1pro_20260918/r1pro_map.yaml").status_code == 200

        for name in ["GLTFLoader", "STLLoader", "OBJLoader", "ColladaLoader", "TGALoader"]:
            assert client.get(f"/vendor/three/addons/loaders/{name}.js").status_code == 200
        assert client.get("/vendor/three/addons/utils/BufferGeometryUtils.js").status_code == 200
        assert client.get("/assets/motphys-logo.svg").content == (ROOT / "frontend/assets/motphys-logo.svg").read_bytes()
        assert "no-cache" in client.get("/js/robot-model-viewer.js").headers["cache-control"]


def test_original_brainco_hand_assets_and_w1_attachment_paths():
    manifest = json.loads((W1_ASSET / "hand-manifest.json").read_text())
    assert len(manifest["hands"]) == 2
    assert len(manifest["files"]) == 26
    main_links = {link.attrib["name"] for link in ET.parse(W1_ASSET / "robot.urdf").findall("link")}
    for hand in manifest["hands"]:
        assert hand["anchor"] in main_links
        assert hand["visualCount"] == 12
        urdf = W1_ASSET / hand["url"].split("dexforce_w1_v030/")[1]
        tree = ET.parse(urdf)
        refs = {node.attrib["filename"] for node in tree.findall(".//visual/geometry/mesh")}
        assert len(refs) == 12
        drivers = {node.attrib["name"] for node in tree.findall("joint") if node.attrib["type"] == "revolute" and node.find("mimic") is None}
        assert drivers == {joint["name"] for joint in hand["joints"]}
        assert len(drivers) == 6 and len(tree.findall(".//mimic")) == 4
    for entry in manifest["files"]:
        blob = (W1_ASSET / entry["path"]).read_bytes()
        assert hashlib.sha256(blob).hexdigest() == entry["sha256"]
        if entry["path"].endswith(".stl"):
            assert len(blob) == 84 + struct.unpack_from("<I", blob, 80)[0] * 50
