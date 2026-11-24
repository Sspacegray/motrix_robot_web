// Pure display precheck; never a hardware authorization or inferred dimension.
export function policyLayout(descriptor, checkpoint) {
  const robotDim = Array.isArray(descriptor?.joints) && descriptor.joints.length ? descriptor.joints.length : null;
  const modelDim = Number.isInteger(checkpoint?.action_dim) && checkpoint.action_dim > 0 ? checkpoint.action_dim : null;
  if (!robotDim || !modelDim) return {text: "Action Layout: 未知 · 缺少预检数据", status: "unknown"};
  const matched = robotDim === modelDim && checkpoint.compatible === true;
  return {text: `Action Layout: 模型 ${modelDim} / 机器人 ${robotDim} DOF · ${matched ? "Mock 维度匹配" : "不匹配或未通过预检"}`, status: matched ? "matched" : "blocked"};
}
