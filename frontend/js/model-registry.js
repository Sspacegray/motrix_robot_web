// Explicit robot identity mapping. Never substitute W1 for Piper or R1 Pro.
export const W1_HANDS = Object.freeze([
  {
    "side": "left",
    "url": "/assets/urdf/dexforce_w1_v030/ee/brainco_revo1/left_hand/left_hand_stl_visual.urdf",
    "anchor": "left_ee",
    "visualCount": 12,
    "embeddedImages": 0,
    "joints": [
      {
        "name": "T_CMC_YAW",
        "lower": 0.0,
        "upper": 1.5708
      },
      {
        "name": "T_MCP",
        "lower": 0.0,
        "upper": 0.8727
      },
      {
        "name": "IF_MCP_PITCH",
        "lower": 0.0,
        "upper": 1.309
      },
      {
        "name": "MF_MCP_PITCH",
        "lower": 0.0,
        "upper": 1.309
      },
      {
        "name": "RF_MCP_PITCH",
        "lower": 0.0,
        "upper": 1.309
      },
      {
        "name": "LF_MCP_PITCH",
        "lower": 0.0,
        "upper": 1.309
      }
    ]
  },
  {
    "side": "right",
    "url": "/assets/urdf/dexforce_w1_v030/ee/brainco_revo1/right_hand/right_hand_stl_visual.urdf",
    "anchor": "right_ee",
    "visualCount": 12,
    "embeddedImages": 0,
    "joints": [
      {
        "name": "T_CMC_YAW",
        "lower": 0.0,
        "upper": 1.5708
      },
      {
        "name": "T_MCP",
        "lower": 0.0,
        "upper": 0.87266
      },
      {
        "name": "IF_MCP_PITCH",
        "lower": 0.0,
        "upper": 1.309
      },
      {
        "name": "MF_MCP_PITCH",
        "lower": 0.0,
        "upper": 1.309
      },
      {
        "name": "RF_MCP_PITCH",
        "lower": 0.0,
        "upper": 1.309
      },
      {
        "name": "LF_MCP_PITCH",
        "lower": 0.0,
        "upper": 1.309
      }
    ]
  }
]);
export const W1_MODEL = Object.freeze({
  robotId: "dex-w1-mock", robotType: "dexforce_w1",
  url: "/assets/urdf/dexforce_w1_v030/robot.urdf",
  visualCount: 21, embeddedImages: 10, hands: W1_HANDS,
});
export function modelForDescriptor(descriptor) {
  return descriptor?.id === W1_MODEL.robotId && descriptor?.robot_type === W1_MODEL.robotType ? W1_MODEL : null;
}
