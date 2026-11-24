"""Offline asset/route validation, not WebGL or browser visual acceptance."""
import hashlib
import json
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET

from fastapi.testclient import TestClient
from motrix_robot_web.app import create_app
from motrix_robot_web.connectors.mock import _w1_descriptor, _r1pro_descriptor, _piper_descriptor

ROOT = Path(__file__).resolve().parents[1]
ASSET = ROOT / "frontend/assets/urdf/dexforce_w1_v030"


def test_w1_visual_mesh_materials_and_embedded_images_are_complete():
    manifest = json.loads((ASSET / "visual-manifest.json").read_text())
    refs = {n.attrib["filename"] for n in ET.parse(ASSET / "robot.urdf").findall(".//visual/geometry/mesh")}
    assert refs == {f["path"] for f in manifest["files"]}
    assert len(refs) == 21
    images = 0
    for record in manifest["files"]:
        blob = (ASSET / record["path"]).read_bytes()
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
    names = {n.attrib["name"] for n in ET.parse(ASSET / "robot.urdf").findall("joint")}
    assert all(j.urdf_joint in names for j in _w1_descriptor().joints)
    assert _w1_descriptor().capabilities.urdf_3d
    assert not _r1pro_descriptor().capabilities.urdf_3d
    assert not _piper_descriptor().capabilities.urdf_3d


def test_offline_model_and_loader_routes_and_fresh_brand(tmp_path):
    app = create_app(data_dir=tmp_path)
    with TestClient(app) as client:
        html = client.get("/")
        assert html.status_code == 200
        assert "LOO1-20261124-map-cloud-v4" in html.text
        assert "motphys-logo.svg?v=" in html.text
        assert '<polygon points="12 2 2 22 22 22"' not in html.text
        assert "no-cache" in html.headers["cache-control"]
        manifest = json.loads((ASSET / "visual-manifest.json").read_text())
        for entry in manifest["files"]:
            response = client.get("/assets/urdf/dexforce_w1_v030/" + entry["path"])
            assert response.status_code == 200
            assert len(response.content) == entry["bytes"]
        for name in ["GLTFLoader", "STLLoader", "ColladaLoader", "TGALoader"]:
            assert client.get(f"/vendor/three/addons/loaders/{name}.js").status_code == 200
        assert client.get("/vendor/three/addons/utils/BufferGeometryUtils.js").status_code == 200
        assert client.get("/assets/motphys-logo.svg").content == (ROOT / "frontend/assets/motphys-logo.svg").read_bytes()
        assert "no-cache" in client.get("/js/robot-model-viewer.js").headers["cache-control"]


def test_original_brainco_hand_assets_and_w1_attachment_paths():
    manifest = json.loads((ASSET / "hand-manifest.json").read_text())
    assert len(manifest["hands"]) == 2
    assert len(manifest["files"]) == 26
    main_links = {link.attrib["name"] for link in ET.parse(ASSET / "robot.urdf").findall("link")}
    for hand in manifest["hands"]:
        assert hand["anchor"] in main_links
        assert hand["visualCount"] == 12
        urdf = ASSET / hand["url"].split("dexforce_w1_v030/")[1]
        tree = ET.parse(urdf)
        refs = {node.attrib["filename"] for node in tree.findall(".//visual/geometry/mesh")}
        assert len(refs) == 12
        drivers = {node.attrib["name"] for node in tree.findall("joint") if node.attrib["type"] == "revolute" and node.find("mimic") is None}
        assert drivers == {joint["name"] for joint in hand["joints"]}
        assert len(drivers) == 6 and len(tree.findall(".//mimic")) == 4
    for entry in manifest["files"]:
        blob = (ASSET / entry["path"]).read_bytes()
        assert hashlib.sha256(blob).hexdigest() == entry["sha256"]
        if entry["path"].endswith(".stl"):
            assert len(blob) == 84 + struct.unpack_from("<I",blob,80)[0] * 50
