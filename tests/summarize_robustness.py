"""Build reviewable reports from the public-browser robustness audit.

Usage: python3 tests/summarize_robustness.py reports/robustness/<run>
No dependency on the robot runtime; this only reads recorded results.
"""

import argparse
import collections
import html
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--plot", action="store_true", help="Also export a standalone outcome chart using matplotlib")
    args = parser.parse_args()
    report = json.loads((args.directory / "results.json").read_text())
    counts = collections.Counter(t["status"] for t in report["trials"])
    valid_trials = [t for t in report["trials"] if t.get("validity") != "interrupted" and t["status"] != "error"]
    valid_counts = collections.Counter(t["status"] for t in valid_trials)
    transports = collections.Counter(t.get("transport", "public-tunnel") for t in valid_trials)
    complete = "finishedAt" in report and not report.get("fatalError") and all(
        sum(t["case"] == c["id"] for t in valid_trials) == report["repeats"]
        for c in report["caseDefinitions"]
    )
    title = "BRACE public demo robustness audit" + ("" if complete else " — IN PROGRESS")
    summary = {"status": "complete" if complete else "in-progress", "url": report["url"],
               "testedCommit": report["commit"], "startedAt": report["startedAt"],
               "finishedAt": report.get("finishedAt"), "attempts": len(report["trials"]),
               "allAttemptOutcomes": dict(counts), "connectedTrials": len(valid_trials),
               "connectedOutcomes": dict(valid_counts), "connectedTransports": dict(transports),
               "interruptedAttempts": len(report["trials"])-len(valid_trials),
               "method": report["method"], "cases": []}
    lines = [f"# {title}", "", f"Demo: {report['url']}", "",
             f"Frozen app commit: `{report['commit']}`. Started {report['startedAt']}.", "",
             f"Recorded attempts: **{len(report['trials'])}**. "
             f"Completed without falling: **{counts['success']}**. "
             f"Falls: **{counts['fall']}**. "
             f"Incomplete tasks without a detected fall: **{counts['incomplete']}**. "
             f"Infrastructure/test errors: **{counts['error']}**.", "",
             f"Connected, valid trials: **{len(valid_trials)}**; "
             f"**{valid_counts['success']}** successes and **{valid_counts['fall']}** falls. "
             "Interrupted attempts are preserved in the overall counts and evidence, "
             "but do not establish policy-only reliability.", "",
             f"Connected-trial transport: **{transports['public-tunnel']} public tunnel**, "
             f"**{transports['isolated-generator']} isolated generator**. "
             "The isolated generator uses the same code, environment, model paths and defaults; "
             "those trials load the published app with its stream override and do not test public network availability.", "",
             "A fall always counts as a failure, including a fall during setup or "
             "a fall followed by recovery. Success additionally requires completion "
             "of the intended task. Errors are not successes and are reported separately.", "",
             "## Per-case results", "",
             "All carrying, exertion, compensation, steering and transition cases use Stealth. "
             "Dumbbells weigh 1 kg each; the barbell is 2 kg total; the kettlebell is 3.5 kg total. "
             "For both-hand force cases, the listed command applies to each hand.", "",
             "| Case | Success / valid trials | Falls in valid trials | Incomplete | Interrupted attempts | Public / isolated trials | Ramp angles | Exertion MAE vs effective |",
             "|---|---:|---:|---:|---:|---|---|---:|"]
    rows = []
    for case in report["caseDefinitions"]:
        attempts = [t for t in report["trials"] if t["case"] == case["id"]]
        trials = [t for t in valid_trials if t["case"] == case["id"]]
        n = collections.Counter(t["status"] for t in trials)
        angles = ", ".join(f"{t['rampAngleDeg']:.2f}°" for t in trials if t.get("rampAngleDeg") is not None)
        # Each trial contributes equally; this is not a pooled error weighted by duration.
        errors = [t["forceMaeN"] for t in trials if t.get("forceMaeN") is not None] if case.get("force", {}).get("mode") == "exert" or case.get("transition") in ["stop-then-push", "settled-push"] else []
        mae = f"{sum(errors) / len(errors):.2f} N" if errors else "—"
        row = [case["id"], f"{n['success']} / {len(trials)}", str(n["fall"]),
               str(n["incomplete"]), str(len(attempts)-len(trials)),
               f"{sum(t.get('transport', 'public-tunnel') == 'public-tunnel' for t in trials)} / {sum(t.get('transport') == 'isolated-generator' for t in trials)}", angles or "—", mae]
        rows.append(row)
        lines.append("| " + " | ".join(row) + " |")
        summary["cases"].append({**case, "validTrials": len(trials), "outcomes": dict(n),
                                 "attempts": len(attempts), "allObservedFalls": sum(t["status"] == "fall" for t in attempts),
                                 "connectedTransports": dict(collections.Counter(t.get("transport", "public-tunnel") for t in trials)),
                                 "rampAnglesDeg": [t["rampAngleDeg"] for t in trials if t.get("rampAngleDeg") is not None],
                                 "exertionMaeN": sum(errors)/len(errors) if errors else None})
    lines += ["", "Force MAE is the mean of the per-trial absolute errors during movement "
              "against the feasible/effective spring command. It is secondary to the fall "
              "criterion and does not imply that the requested force was fully attainable.", "",
              "## Failure and incomplete evidence", ""]
    exceptions = [t for t in report["trials"] if t["status"] != "success"]
    if not exceptions:
        lines += ["No exceptions in the recorded attempts.", ""]
    for trial in exceptions:
        fall = trial.get("fall") or {}
        detail = (f"{fall.get('reason')} at {fall.get('elapsed', 0):.2f} simulated seconds "
                  f"during {fall.get('phase')}" if fall else trial.get("error") or
                  "Intended task did not complete; inspect the trace and video.")
        if trial.get("review"):
            detail += ". " + trial["review"]
        if trial.get("validity") == "interrupted":
            detail += ". Interrupted attempt; attribution to the policy is inconclusive"
        links = [f"[trace](traces/{trial['id']}.json)"]
        for field in ["video", "screenshot"]:
            if trial.get(field):
                links.append(f"[{field}]({trial[field]})")
        lines += [f"- **{trial['id']} — {trial['status']}**: {detail}. " + "; ".join(links)]
    lines += ["", "## Method and scope", ""]
    for key, value in report["method"].items():
        lines += [f"**{key.capitalize()}:** {value}", ""]
    lines += ["The test freezes downloaded public app assets and records their SHA-256 hashes "
              "in results.json. A read-only hook observes MuJoCo state in the browser; "
              "the physics, policy and control parameters are unchanged. Videos preserve "
              "the public UI, while traces record pose and local terrain clearance.", "",
              "These are short observed trials, not proof of failure-free operation. "
              "The physical equipment uses the demo's ideal gripping model; object slipping "
              "or release is outside this audit. Simulated time is reported separately "
              "from wall-clock time.", "",
              "## Every trial", "",
              "| Trial | Result | Validity | Simulated duration | Min pelvis clearance | Video |",
              "|---|---|---|---:|---:|---|"]
    for t in report["trials"]:
        duration = f"{t['simulatedSeconds']:.2f} s" if t.get("simulatedSeconds") is not None else "—"
        clearance = f"{t['minPelvisClearance']:.3f} m" if t.get("minPelvisClearance") is not None else "—"
        video = f"[view]({t['video']})" if t.get("video") else "—"
        lines.append(f"| {t['id']} | {t['status']} | {t.get('validity', 'valid')} | {duration} | {clearance} | {video} |")
    (args.directory / "REPORT.md").write_text("\n".join(lines) + "\n")
    (args.directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    # A standalone review page keeps videos local and exposes all outcomes.
    def esc(value):
        return html.escape(str(value), quote=True)

    table = "".join("<tr>" + "".join(f"<td>{esc(v)}</td>" for v in row) + "</tr>" for row in rows)
    cards = []
    for t in report["trials"]:
        video = (f'<video controls preload="none" src="{esc(t["video"])}"></video>'
                 if t.get("video") else "")
        fall = t.get("fall")
        detail = (json.dumps(fall, indent=2) if fall else t.get("error", "")) + "\n" + t.get("review", "")
        cards.append(f'<article class="{esc(t["status"])}" data-status="{esc(t["status"])}" data-validity="{esc(t.get("validity", "valid"))}"><h3>{esc(t["id"])}</h3>'
                     f'<p><strong>{esc(t["status"])}</strong> · {esc(t.get("validity", "valid"))}</p>{video}'
                     f'<pre>{esc(detail)}</pre><a href="traces/{esc(t["id"])}.json">Trace</a></article>')
    method = "".join(f"<p><b>{esc(k)}:</b> {esc(v)}</p>" for k, v in report["method"].items())
    page = f'''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>{esc(title)}</title>
<style>body{{font:16px system-ui;background:#f4f8fc;color:#163149;max-width:1250px;margin:auto;padding:24px}}
table{{border-collapse:collapse;width:100%;background:white;font-size:14px}}th,td{{padding:9px;border-bottom:1px solid #dce7f1;text-align:left}}
.scroll{{overflow:auto}}.cards{{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:18px}}
article{{background:white;border:1px solid #cbdde9;border-radius:12px;padding:16px}}article.fall{{border:2px solid #ba4444}}
video{{width:100%;border-radius:8px}}pre{{white-space:pre-wrap;font-size:12px}}h3{{font-size:15px;overflow-wrap:anywhere}}
a{{color:#275e93}}.summary{{font-size:20px}}h1{{font-size:30px}}</style>
<h1>{esc(title)}</h1><p><a href="{esc(report['url'])}">Published demo</a> · Commit {esc(report['commit'][:12])}</p>
<p class="summary">{len(report['trials'])} attempts · {counts['success']} successes · {counts['fall']} falls · {counts['incomplete']} incomplete · {counts['error']} errors</p>
<p>Connected trials: {len(valid_trials)} · {valid_counts['success']} successes · {valid_counts['fall']} falls. Interrupted attempts remain in the overall counts and videos.</p>
<p><b>Transport:</b> {transports['public-tunnel']} public-tunnel trials; {transports['isolated-generator']} trials with the published app and an isolated copy of the same generator. Isolated trials do not test public network availability.</p>
<p>Every fall is a failure, even if the robot recovers. Success also requires task completion. This small sample does not establish guaranteed reliability.</p>
<p>All non-gait cases use Stealth. Dumbbells: 1 kg each. Barbell: 2 kg total. Kettlebell: 3.5 kg total. Both-hand force values are per hand.</p>
<div class="scroll"><table><thead><tr><th>Case</th><th>Success / valid trials</th><th>Falls</th><th>Incomplete</th><th>Interrupted attempts</th><th>Public / isolated</th><th>Angles</th><th>Exertion MAE</th></tr></thead><tbody>{table}</tbody></table></div>
<p>Force MAE is measured relative to effective force; it is secondary to the fall criterion.</p>
<h2>Recorded trials</h2><p><label>Outcome <select id="outcome"><option value="all">All attempts</option><option value="success">Successes</option><option value="fall">Falls</option><option value="incomplete">Incomplete</option><option value="interrupted">Interrupted</option></select></label>
<label> Case <input id="case" type="search" placeholder="e.g. kettlebell or stop-then-push"></label></p>
<div class="cards">{''.join(cards)}</div><h2>Method</h2>{method}
<p><a href="REPORT.md">Full Markdown report</a> · <a href="results.json">Results and asset hashes</a></p>
<script>const outcome=document.querySelector('#outcome'),query=document.querySelector('#case');
function filter(){{for(const card of document.querySelectorAll('.cards article')){{const match=outcome.value==='all'||(outcome.value==='interrupted'?card.dataset.validity==='interrupted':card.dataset.status===outcome.value);card.hidden=!match||!card.querySelector('h3').textContent.toLowerCase().includes(query.value.toLowerCase());}}}}
outcome.addEventListener('change',filter);query.addEventListener('input',filter);</script></html>'''
    (args.directory / "index.html").write_text(page)
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np

        names = [c["id"] for c in report["caseDefinitions"]]
        outcomes = [collections.Counter(t["status"] for t in valid_trials if t["case"] == name) for name in names]
        fig, ax = plt.subplots(figsize=(12, 14), layout="constrained")
        fig.set_facecolor("#f5f9fd")
        ax.set_facecolor("#ffffff")
        positions = np.arange(len(names))
        left = np.zeros(len(names))
        for status, label, color in [("success", "Completed without falling", "#3e8c78"),
                                     ("fall", "Fall — failure", "#ba5252"),
                                     ("incomplete", "No fall; task incomplete", "#d39b40")]:
            values = np.array([n[status] for n in outcomes])
            ax.barh(positions, values, left=left, label=label, color=color, height=.65)
            left += values
        ax.set_yticks(positions, [name.replace("-", " ") for name in names], fontsize=10)
        ax.invert_yaxis()
        ax.set_xlim(0, max(report["repeats"], max(left, default=0)) + .6)
        ax.set_xticks(range(report["repeats"] + 1))
        ax.set_xlabel("Connected trials; each fall remains a failed trial")
        for i, n in enumerate(outcomes):
            ax.text(left[i] + .08, i, f"{n['success']}/{int(left[i])}", va="center", fontsize=10)
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="x", alpha=.12)
        ax.set_axisbelow(True)
        ax.legend(loc="lower right", fontsize=9)
        ax.set_title("BRACE public demo — observed robustness" + ("" if complete else " (in progress)") + "\n"
                     f"{len(valid_trials)} connected trials · {valid_counts['success']} successes · {valid_counts['fall']} falls\n"
                     f"{transports['public-tunnel']} public / {transports['isolated-generator']} isolated · {len(report['trials']) - len(valid_trials)} interrupted attempts retained separately\nSmall sample; no reliability guarantee",
                     loc="left", fontsize=14, pad=20)
        fig.savefig(args.directory / "outcomes.png", dpi=160)
        plt.close(fig)
        examples = [("exert-right-forward-5N-slope-r3", "Forward 5 N on slope"),
                    ("exert-right-downward-5N-flat-r3", "Downward 5 N while walking"),
                    ("transition-stop-then-push-r2", "Stop, then push 5 N — failed trial")]
        example_trials = [(next((t for t in valid_trials if t["id"] == trial_id), None), label)
                          for trial_id, label in examples]
        example_trials = [(t, label) for t, label in example_trials if t]
        if example_trials:
            fig, axes = plt.subplots(len(example_trials), 1, figsize=(11, 3.1*len(example_trials)), layout="constrained", squeeze=False)
            for ax, (trial, label) in zip(axes[:, 0], example_trials):
                trace = json.loads((args.directory / "traces" / f"{trial['id']}.json").read_text())
                samples = [s for s in trace["samples"] if s["phase"] == "motion"]
                origin = samples[0]["t"]
                times = [s["t"]-origin for s in samples]
                ax.plot(times, [s["effective"][1] for s in samples], color="#35658c", label="Effective target", linewidth=2)
                ax.plot(times, [s["measured"][1] for s in samples], color="#b17a2f", label="Measured reaction", linewidth=1.4)
                if trial.get("fall"):
                    ax.axvline(trial["fall"]["elapsed"]-origin, color="#b44949", linestyle="--", label="Fall detected")
                ax.set_title(f"{label} · control-step MAE {trial['forceMaeN']:.2f} N", loc="left")
                ax.set_ylabel("Right-hand force (N)")
                ax.set_xlabel("Time since movement phase began (simulated seconds)")
                ax.legend(loc="upper left", fontsize=9)
                ax.grid(alpha=.15)
                ax.spines[["top", "right"]].set_visible(False)
            fig.suptitle("Force accuracy is separate from fall-free balance\nTraces shown at approximately 5 Hz; MAE uses every control event (50 Hz)", fontsize=14)
            fig.savefig(args.directory / "force-traces.png", dpi=160)
            plt.close(fig)
    print(json.dumps(dict(counts), sort_keys=True))


if __name__ == "__main__":
    main()
