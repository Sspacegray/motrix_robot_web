// Event boundary between the editable 2D canvas and read-only 3D camera.
// No robot API, Lease, or goal dispatch imports.
export function bindMapInteractions({canvas, cloudCanvas, windowTarget, isEnabled, onDown, onMove, onUp, onWheel}) {
  let active = false;
  const bindings = [];
  const listen = (target, type, handler, options) => {
    target.addEventListener(type, handler, options);
    bindings.push(() => target.removeEventListener(type, handler, options));
  };
  const fromCloud = event => event.target === cloudCanvas || (event.composedPath?.() || []).includes(cloudCanvas);
  const reset = () => { active = false; onUp(); };
  listen(canvas, "mousedown", event => {
    if (!isEnabled() || fromCloud(event)) { reset(); return; }
    if (event.button !== 0 && event.button !== 2) return;
    active = true;
    onDown(event);
  });
  listen(windowTarget, "mousemove", event => {
    if (!isEnabled() || fromCloud(event)) { reset(); return; }
    if (active && "buttons" in event && event.buttons === 0) { reset(); return; }
    if (active || event.target === canvas) onMove(event);
  });
  listen(windowTarget, "mouseup", reset);
  listen(windowTarget, "blur", reset);
  listen(canvas, "wheel", event => {
    if (!isEnabled() || fromCloud(event)) { reset(); return; }
    onWheel(event);
  }, {passive:false});
  listen(canvas, "contextmenu", event => { if (isEnabled()) event.preventDefault(); });
  // Also clear any unfinished 2D drag before OrbitControls handles 3D input.
  for (const type of ["pointerdown", "mousedown", "wheel", "click"]) listen(cloudCanvas, type, reset, {passive:true});
  return {reset, destroy() { reset(); bindings.forEach(remove => remove()); }};
}
