// Offline job helpers: escape metadata and show explicit Asia/Shanghai time.
export function escapeHtml(value) {
  return String(value ?? "").replace(/[&<>"']/g, c => ({"&":"&amp;", "<":"&lt;", ">":"&gt;", '"':"&quot;", "'":"&#39;"}[c]));
}
export function formatLocalTime(value) {
  if (!value) return "—";
  const date = new Date(value);
  if (Number.isNaN(date.getTime())) return "—";
  return new Intl.DateTimeFormat("zh-CN", {timeZone:"Asia/Shanghai", month:"2-digit", day:"2-digit", hour:"2-digit", minute:"2-digit", second:"2-digit", hour12:false}).format(date);
}
export function filterJobs(jobs, state = "ALL", query = "") {
  const needle = query.trim().toLowerCase();
  return jobs.filter(job => (state === "ALL" || job.state === state) &&
    [job.flow_name, job.run_id, job.operator, job.job_id].some(v => String(v ?? "").toLowerCase().includes(needle)));
}
