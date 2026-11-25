// Motphys Robot Web — Accessible Toast Notifications & Native Dialog Helpers
// Zero external dependencies; follows modern-web-guidance (dialog, light-dismiss-a-dialog, persistent-toast-notifications).

let toastRegion = null;
let liveRegion = null;

function ensureToastRegion() {
  if (!toastRegion) {
    toastRegion = document.createElement("div");
    toastRegion.id = "toastRegion";
    toastRegion.className = "toast-container";
    toastRegion.setAttribute("role", "region");
    toastRegion.setAttribute("aria-label", "Notifications");
    document.body.appendChild(toastRegion);
  }
  if (!liveRegion) {
    liveRegion = document.createElement("div");
    liveRegion.id = "srLiveRegion";
    liveRegion.className = "sr-only";
    liveRegion.setAttribute("aria-live", "polite");
    liveRegion.setAttribute("aria-atomic", "true");
    document.body.appendChild(liveRegion);
  }
  return toastRegion;
}

function announce(text, priority = "normal") {
  if (typeof document.ariaNotify === "function") {
    try {
      document.ariaNotify(text, { priority });
      return;
    } catch {
      // Fall through to live region
    }
  }
  if (liveRegion) {
    liveRegion.textContent = "";
    requestAnimationFrame(() => {
      if (liveRegion) liveRegion.textContent = text;
    });
  }
}

export function notify(message, type = "info", durationMs = 3600) {
  if (typeof document === "undefined") return;
  const region = ensureToastRegion();
  const normalizedType = type === "danger" ? "error" : type;

  const toast = document.createElement("div");
  toast.className = `toast ${normalizedType}`;

  const textSpan = document.createElement("span");
  textSpan.className = "toast-text";
  textSpan.textContent = String(message);

  const closeBtn = document.createElement("button");
  closeBtn.type = "button";
  closeBtn.className = "toast-close";
  closeBtn.setAttribute("aria-label", "关闭通知");
  closeBtn.textContent = "✕";

  toast.appendChild(textSpan);
  toast.appendChild(closeBtn);
  // Keep stack compact (max 2 concurrent toasts) so controls are never occluded
  while (region.children.length >= 2) {
    region.firstElementChild?.remove();
  }
  region.appendChild(toast);

  announce(String(message), normalizedType === "error" ? "high" : "normal");

  let timer = null;
  let paused = false;

  const dismiss = () => {
    if (timer) clearTimeout(timer);
    if (!toast.isConnected) return;
    const wasFocused = toast.contains(document.activeElement);
    const nextToast = toast.nextElementSibling || toast.previousElementSibling;
    toast.classList.add("toast-leaving");
    setTimeout(() => {
      toast.remove();
      if (wasFocused) nextToast?.querySelector?.(".toast-close")?.focus?.();
    }, 160);
  };

  const armTimer = () => {
    if (timer) clearTimeout(timer);
    if (durationMs > 0 && !paused) {
      timer = setTimeout(dismiss, durationMs);
    }
  };

  closeBtn.addEventListener("click", dismiss);
  toast.addEventListener("mouseenter", () => {
    paused = true;
    if (timer) clearTimeout(timer);
  });
  toast.addEventListener("mouseleave", () => {
    paused = false;
    armTimer();
  });
  toast.addEventListener("focusin", () => {
    paused = true;
    if (timer) clearTimeout(timer);
  });
  toast.addEventListener("focusout", () => {
    paused = false;
    armTimer();
  });

  armTimer();
  return dismiss;
}

/**
 * Progressive-enhancement light-dismiss fallback for <dialog closedby="any">
 * in browsers lacking HTMLDialogElement.prototype.closedBy (e.g. Safari).
 * Also resets dialog.returnValue on close to prevent stale returnValue leaks.
 */
export function enableDialogLightDismiss(dialog) {
  if (!dialog) return;
  dialog.addEventListener("close", () => {
    dialog.returnValue = "";
  });

  if (typeof HTMLDialogElement !== "undefined" && !("closedBy" in HTMLDialogElement.prototype)) {
    const outside = (e) => {
      const r = dialog.getBoundingClientRect();
      return e.clientX < r.left || e.clientX > r.right || e.clientY < r.top || e.clientY > r.bottom;
    };
    let pressedOutside = false;
    dialog.addEventListener("pointerdown", (e) => {
      pressedOutside = outside(e);
    });
    dialog.addEventListener("pointerup", (e) => {
      if (pressedOutside && outside(e)) {
        dialog.close("dismiss");
      }
      pressedOutside = false;
    });
  }
}
