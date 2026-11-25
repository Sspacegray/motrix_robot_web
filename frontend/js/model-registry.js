// Explicit robot identity mapping. Never substitute one robot's URDF/model for another.
export const W1_HANDS = Object.freeze([
  {
    side: "left",
    url: "/assets/urdf/dexforce_w1_v030/ee/brainco_revo1/left_hand/left_hand_stl_visual.urdf",
    anchor: "left_ee",
    visualCount: 12,
    embeddedImages: 0,
    joints: [
      { name: "T_CMC_YAW", lower: 0.0, upper: 1.5708 },
      { name: "T_MCP", lower: 0.0, upper: 0.8727 },
      { name: "IF_MCP_PITCH", lower: 0.0, upper: 1.309 },
      { name: "MF_MCP_PITCH", lower: 0.0, upper: 1.309 },
      { name: "RF_MCP_PITCH", lower: 0.0, upper: 1.309 },
      { name: "LF_MCP_PITCH", lower: 0.0, upper: 1.309 },
    ],
  },
  {
    side: "right",
    url: "/assets/urdf/dexforce_w1_v030/ee/brainco_revo1/right_hand/right_hand_stl_visual.urdf",
    anchor: "right_ee",
    visualCount: 12,
    embeddedImages: 0,
    joints: [
      { name: "T_CMC_YAW", lower: 0.0, upper: 1.5708 },
      { name: "T_MCP", lower: 0.0, upper: 0.87266 },
      { name: "IF_MCP_PITCH", lower: 0.0, upper: 1.309 },
      { name: "MF_MCP_PITCH", lower: 0.0, upper: 1.309 },
      { name: "RF_MCP_PITCH", lower: 0.0, upper: 1.309 },
      { name: "LF_MCP_PITCH", lower: 0.0, upper: 1.309 },
    ],
  },
]);

export const W1_MODEL = Object.freeze({
  robotId: "dex-w1-mock",
  robotType: "dexforce_w1",
  label: "Dexforce W1 原始 URDF",
  formatLabel: "GLB",
  url: "/assets/urdf/dexforce_w1_v030/robot.urdf",
  visualCount: 21,
  embeddedImages: 10,
  preserveGltfPbr: true,
  defaultViewDir: [1.2, 0.65, 1.4],
  trailLinks: Object.freeze([
    { name: "left_ee", color: 0x00f0ff, lightColor: 0x0284c7, label: "L腕" },
    { name: "right_ee", color: 0xff2e93, lightColor: 0xdb2777, label: "R腕" },
    { name: "head_pitch_j2_link", color: 0xfacc15, lightColor: 0xd97706, label: "头" },
    { name: "ankle_pitch_link", color: 0x22c55e, lightColor: 0x059669, label: "脚" },
  ]),
  hands: W1_HANDS,
});

export const R1PRO_MODEL = Object.freeze({
  robotId: "r1pro-mock",
  robotType: "r1pro",
  label: "Galaxea R1 Pro 官方贴图 URDF",
  formatLabel: "OBJ",
  url: "/assets/urdf/galaxea_r1pro/robot.urdf",
  visualCount: 36,
  embeddedImages: 1,
  preserveGltfPbr: false,
  pbrProfile: "r1pro",
  defaultViewDir: [1.35, 0.55, 1.35],
  trailLinks: Object.freeze([
    { name: "left_gripper_link", color: 0x00f0ff, lightColor: 0x0284c7, label: "L夹爪" },
    { name: "right_gripper_link", color: 0xff2e93, lightColor: 0xdb2777, label: "R夹爪" },
    { name: "zed_link", color: 0xfacc15, lightColor: 0xd97706, label: "头" },
  ]),
  hands: Object.freeze([]),
});

export const PIPER_MODEL = Object.freeze({
  robotId: "piper-dual-mock",
  robotType: "piper_dual",
  label: "AgileX Piper Dual + D405 PBR URDF",
  formatLabel: "STL/OBJ",
  url: "/assets/urdf/piper_dual_d405/robot.urdf",
  visualCount: 77,
  embeddedImages: 1,
  preserveGltfPbr: false,
  pbrProfile: "piper_dual",
  defaultViewDir: [1.35, 0.95, 1.15],
  trailLinks: Object.freeze([
    { name: "left_gripper_base", color: 0x00f0ff, lightColor: 0x0284c7, label: "L夹爪" },
    { name: "right_gripper_base", color: 0xff2e93, lightColor: 0xdb2777, label: "R夹爪" },
  ]),
  hands: Object.freeze([]),
});

export const ROBOT_MODELS = Object.freeze([W1_MODEL, R1PRO_MODEL, PIPER_MODEL]);

function parseHexColor(value, fallback) {
  if (typeof value === "number" && Number.isFinite(value)) return value;
  if (typeof value === "string") {
    const clean = value.trim().replace(/^#/, "").replace(/^0x/i, "");
    const parsed = Number.parseInt(clean, 16);
    if (Number.isFinite(parsed)) return parsed;
  }
  return fallback;
}

export function modelForDescriptor(descriptor) {
  if (!descriptor) return null;
  const builtin = ROBOT_MODELS.find(
    (m) => descriptor.id === m.robotId && descriptor.robot_type === m.robotType
  );
  if (builtin) return builtin;

  const profile = descriptor.model_profile;
  if (!profile?.url || (!profile.url.startsWith("/custom-robots/") && !profile.url.startsWith("/assets/"))) {
    return null;
  }
  const paletteDark = [0x00f0ff, 0xff2e93, 0xfacc15, 0x22c55e];
  const paletteLight = [0x0284c7, 0xdb2777, 0xd97706, 0x059669];
  const trailLinks = (profile.trail_links || [])
    .filter((t) => t && t.name)
    .map((t, idx) => ({
      name: t.name,
      color: parseHexColor(t.color, paletteDark[idx % paletteDark.length]),
      lightColor: parseHexColor(t.light_color ?? t.lightColor, paletteLight[idx % paletteLight.length]),
      label: t.label || t.name,
    }));

  return {
    robotId: descriptor.id,
    robotType: descriptor.robot_type,
    label: profile.label || `${descriptor.name} URDF`,
    formatLabel: profile.format_label || "URDF",
    url: profile.url,
    visualCount: Number(profile.visual_count) || 0,
    embeddedImages: Number(profile.embedded_images) || 0,
    preserveGltfPbr: Boolean(profile.preserve_gltf_pbr),
    pbrProfile: profile.pbr_profile || (profile.preserve_gltf_pbr ? null : "custom"),
    defaultViewDir: Array.isArray(profile.default_view_dir) && profile.default_view_dir.length >= 3
      ? profile.default_view_dir
      : [1.35, 0.7, 1.35],
    trailLinks,
    hands: Array.isArray(profile.hands) ? profile.hands : [],
  };
}
