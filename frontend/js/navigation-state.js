// One visible global entry per feature; capabilities disable, never erase navigation.
export const FEATURE_LABELS = Object.freeze({overview:"总览", actions:"动作与工作流", joints:"关节 / 3D", navigation:"地图导航", perception:"视觉感知", policy:"策略评测", diagnostics:"安全诊断"});
export function featureAvailability(caps = {}) {
  const supported = {overview:true, diagnostics:true, actions:!!(caps.atomic_actions || caps.workflows), joints:!!caps.joints,
    navigation:!!caps.navigation, perception:!!(caps.cameras || caps.head_depth || caps.yolo_foundation_pose || caps.aruco), policy:!!caps.policy_inference};
  const reasons = {navigation:"当前机型无地图导航/底盘能力；请切换 R1 Pro Mock 查看", policy:"当前机型未提供策略推理；请切换 R1 Pro 或 Piper Mock 查看", perception:"当前机型未提供相机或感知能力", joints:"当前机型未提供关节状态", actions:"当前机型未提供原子动作或工作流能力"};
  return Object.fromEntries(Object.entries(FEATURE_LABELS).map(([name,label]) => [name,{label,enabled:supported[name],reason:supported[name] ? "" : reasons[name]}]));
}
export function applyFeatureNavigation(buttons, availability) {
  for (const button of buttons) {
    const feature = availability[button.getAttribute("data-tab")];
    if (!feature) continue;
    button.hidden = false;
    button.style.display = "inline-flex";
    button.disabled = !feature.enabled;
    button.setAttribute("aria-disabled", String(!feature.enabled));
    button.removeAttribute("aria-hidden");
    button.title = feature.enabled ? feature.label : feature.reason;
    button.dataset.disabledReason = feature.reason;
  }
}
