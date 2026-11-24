// Explicit robot identity mapping. Never substitute W1 for Piper or R1 Pro.
export const W1_MODEL = Object.freeze({
  robotId: "dex-w1-mock", robotType: "dexforce_w1",
  url: "/assets/urdf/dexforce_w1_v030/robot.urdf",
  visualCount: 21, embeddedImages: 10,
});
export function modelForDescriptor(descriptor) {
  return descriptor?.id === W1_MODEL.robotId && descriptor?.robot_type === W1_MODEL.robotType ? W1_MODEL : null;
}
