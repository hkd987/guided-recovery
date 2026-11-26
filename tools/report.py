#!/usr/bin/env python3
"""Turn grsim/grreplay outputs into a single self-contained HTML report.

    python3 tools/report.py reports reports/index.html

No third-party packages: plots are inline SVG.
"""
import csv
import html
import json
import math
import os
import sys


def load_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def load_csv(path):
    try:
        with open(path) as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def scatter_svg(runs, radius=30.0, size=360, span=120.0):
    """Landing points relative to the pad, nominal runs only."""
    c = size / 2
    k = c / span
    parts = [f'<svg viewBox="0 0 {size} {size}" width="{size}" height="{size}" role="img" '
             f'aria-label="Landing points relative to the pad">',
             f'<rect width="{size}" height="{size}" fill="var(--panel)"/>']
    for r_m, dash in ((radius, ""), (2 * radius, "4 4"), (span, "2 6")):
        parts.append(f'<circle cx="{c}" cy="{c}" r="{r_m * k:.1f}" fill="none" stroke="var(--muted)" '
                     f'stroke-dasharray="{dash}"/>')
    outside = 0
    for r in runs:
        if r.get("fault") not in ("none", "gust", "shear") or r.get("landed") != "1":
            continue
        n, e = float(r["land_n"]), float(r["land_e"])
        if math.hypot(n, e) > span:
            outside += 1
            continue
        color = "var(--good)" if math.hypot(n, e) <= radius else "var(--bad)"
        parts.append(f'<circle cx="{c + e * k:.1f}" cy="{c - n * k:.1f}" r="2" fill="{color}" opacity="0.7"/>')
    parts.append(f'<text x="8" y="16" class="lbl">N up, rings {radius:.0f} / {2 * radius:.0f} / {span:.0f} m</text>')
    if outside:
        parts.append(f'<text x="8" y="{size - 8}" class="lbl">{outside} beyond {span:.0f} m not shown</text>')
    parts.append("</svg>")
    return "".join(parts)


def hist_svg(values, edges, width=420, height=180, label="miss distance (m)"):
    counts = [0] * (len(edges) - 1)
    for v in values:
        for i in range(len(edges) - 1):
            if edges[i] <= v < edges[i + 1] or (i == len(edges) - 2 and v >= edges[-1]):
                counts[i] += 1
                break
    top = max(counts) or 1
    bw = (width - 40) / len(counts)
    parts = [f'<svg viewBox="0 0 {width} {height}" width="{width}" height="{height}" role="img" aria-label="{label}">']
    for i, cnt in enumerate(counts):
        h = (height - 40) * cnt / top
        x = 30 + i * bw
        parts.append(f'<rect x="{x + 1:.1f}" y="{height - 25 - h:.1f}" width="{bw - 2:.1f}" height="{h:.1f}" '
                     f'fill="var(--accent)"/>')
        parts.append(f'<text x="{x + bw / 2:.1f}" y="{height - 10}" class="lbl" text-anchor="middle">'
                     f'{edges[i]:g}</text>')
    parts.append(f'<text x="{width - 4}" y="12" class="lbl" text-anchor="end">{label}</text></svg>')
    return "".join(parts)


def summary_rows(s):
    if not s:
        return "<tr><td colspan=2>not run</td></tr>"
    rows = [("runs", s["runs"]), ("landed", s["landed"]), (f"within {s['radius_m']:.0f} m", f"{s['within_pct']:.1f}%"),
            ("median miss (CEP50)", f"{s['cep50_m']:.1f} m"), ("90th percentile", f"{s['p90_m']:.1f} m"),
            ("worst", f"{s['max_miss_m']:.1f} m"), ("canopy never confirmed", s["not_confirmed"]),
            ("steering before canopy", s["nav_before_canopy"]), ("crashes", s["crashed"]),
            ("mean confirm delay", f"{s['confirm_delay_mean_s']:.2f} s")]
    return "".join(f"<tr><td>{html.escape(str(a))}</td><td>{html.escape(str(b))}</td></tr>" for a, b in rows)


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "reports"
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(src, "index.html")
    g1 = load_json(os.path.join(src, "gate1_summary.json"))
    g2 = load_json(os.path.join(src, "gate2_summary.json"))
    runs2 = load_csv(os.path.join(src, "gate2_runs.csv"))
    g3 = load_csv(os.path.join(src, "gate3.csv"))

    def badge(ok):
        return '<span class="pass">PASS</span>' if ok else '<span class="fail">FAIL</span>'

    faults = ""
    if g2:
        for name, f in sorted(g2["summary"].get("faults", {}).items()):
            faults += (f"<tr><td>{html.escape(name)}</td><td>{f['runs']}</td><td>{f['exercised']}</td>"
                       f"<td>{f['met']}/{f['exercised']}</td></tr>")
    misses = [float(r["miss"]) for r in runs2 if r.get("fault") == "none" and r.get("landed") == "1"]
    g3_main = [r for r in g3 if r["has_main"] == "1"]
    g3_det = sum(1 for r in g3_main if r["detected"] == "1" and r["missed"] == "0" and r["false_trigger"] == "0")
    g3_false = sum(1 for r in g3 if r["false_trigger"] == "1")
    g3_ok = bool(g3) and g3_false == 0 and g3_det == len(g3_main)

    doc = f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Gate Report</title>
<style>
:root {{ --bg:#fbfaf7; --panel:#f1efe9; --ink:#1d1d1b; --muted:#8a877f; --accent:#3d6b8c; --good:#2f7d4f; --bad:#b4442f; }}
@media (prefers-color-scheme: dark) {{ :root {{ --bg:#16171a; --panel:#202226; --ink:#e8e6e1; --muted:#7d7a73; --accent:#7fb0d4; --good:#5fbf86; --bad:#e07a62; }} }}
body {{ background:var(--bg); color:var(--ink); font:15px/1.5 system-ui, sans-serif; margin:0 auto; max-width:900px; padding:24px 16px; }}
table {{ border-collapse:collapse; margin:8px 0 20px; }} td,th {{ padding:4px 12px 4px 0; text-align:left; border-bottom:1px solid var(--panel); }}
.pass {{ color:var(--good); font-weight:600; }} .fail {{ color:var(--bad); font-weight:600; }}
.lbl {{ fill:var(--muted); font-size:11px; }} .row {{ display:flex; flex-wrap:wrap; gap:24px; }} svg {{ max-width:100%; height:auto; }}
</style></head><body>
<h1>Guided Recovery Pod: software gates</h1>
<h2>Gate 1: guidance in a simple model {badge(bool(g1 and g1.get('pass')))}</h2>
<table>{summary_rows(g1 and g1.get('summary'))}</table>
<h2>Gate 2: software in the loop {badge(bool(g2 and g2.get('pass')))}</h2>
"""
    if g2:
        doc += f"<p>Same seeds through gate 1: {g2['gate1_same_seeds']['within_pct']:.1f}% within 30 m.</p>"
        doc += f"<table>{summary_rows(g2['summary'])}</table>"
        doc += "<h3>Injected faults</h3><table><tr><th>fault</th><th>runs</th><th>exercised</th><th>expected outcome</th></tr>"
        doc += faults + "</table>"
        if g2.get("failures"):
            doc += "<p class='fail'>" + "<br>".join(html.escape(f) for f in g2["failures"]) + "</p>"
        doc += '<div class="row">' + scatter_svg(runs2) + hist_svg(misses, [0, 5, 10, 15, 20, 30, 50, 100, 200]) + "</div>"
    doc += f"<h2>Gate 3: log replay {badge(g3_ok)}</h2>"
    doc += (f"<p>{len(g3)} logs, {len(g3_main)} with a main deployment: {g3_det} detected, "
            f"{g3_false} false triggers.</p>" if g3 else "<p>not run</p>")
    doc += "</body></html>"
    with open(out, "w") as f:
        f.write(doc)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
