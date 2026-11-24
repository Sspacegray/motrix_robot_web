// Pure local state: never import API, write feedback, or dispatch robot actions.
export class PosePreview {
  constructor() { this.robotId = null; this.targets = {}; this.feedback = {}; this.handTargets = {}; this.bodySpecs = []; this.handSpecs = []; this.mode = "target"; }
  selectRobot(descriptor, observation = {}, handSpecs = []) {
    this.robotId = descriptor.id; this.bodySpecs = descriptor.joints || []; this.handSpecs = handSpecs;
    this.feedback = {...(observation.joint_positions || {})};
    this.targets = Object.fromEntries(this.bodySpecs.map(joint => [joint.name, this.feedback[joint.name] ?? joint.default ?? 0]));
    this.handTargets = Object.fromEntries(handSpecs.map(hand => [hand.side, Object.fromEntries(hand.joints.map(joint => [joint.name,joint.lower]))]));
    this.mode = "target";
  }
  observe(robotId, positions) { if (robotId === this.robotId) this.feedback = {...positions}; }
  setTarget(name, value) {
    const spec = this.bodySpecs.find(joint => joint.name === name);
    if (!spec || !Number.isFinite(value)) return false;
    this.targets[name] = Math.min(spec.upper, Math.max(spec.lower,value)); this.mode = "target"; return true;
  }
  setHandTarget(side, name, value) {
    const spec = this.handSpecs.find(hand => hand.side === side)?.joints.find(joint => joint.name === name);
    if (!spec || !Number.isFinite(value)) return false;
    this.handTargets[side][name] = Math.min(spec.upper, Math.max(spec.lower,value)); return true;
  }
  copyFeedback() { for(const joint of this.bodySpecs) if(Number.isFinite(this.feedback[joint.name])) this.targets[joint.name]=this.feedback[joint.name]; this.mode="target"; }
  visiblePose() { return {...(this.mode === "feedback" ? this.feedback : this.targets)}; }
  commandTargets() { return {...this.targets}; } // Hand preview keys never enter body command payload.
}
export function bindLocalPreviewInput(input, apply, render) {
  input.addEventListener("input", event => { if(apply(Number(event.target.value))) render(); });
}

export function applyModelJoints(robot, descriptor, positions) {
  if (!robot || !descriptor) return;
  for (const joint of descriptor.joints || []) {
    const value = positions[joint.name], name = joint.urdf_joint || joint.name;
    if(Number.isFinite(value) && robot.joints[name]) robot.setJointValue(name,value);
  }
}
