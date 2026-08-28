"""Build the shareable HTML evaluation report (self-contained, no CDN, works offline).

`build_report(runs, scene)` -> dict; `render(report, path)` -> writes the HTML.
The template is a single file with a __FLIGHTVLA_DATA__ token replaced by the
report JSON; everything (3D playback, charts, keyframes, tables) is vanilla JS
canvas so the file can be attached to an issue or dropped in a chat.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import List, Optional

from . import __version__
from .metrics import compare_metrics


def build_report(runs: List[dict], scene, title: str = "FlightVLA Guard Report") -> dict:
    return {
        "generator": f"flightvla {__version__}",
        "title": title,
        "scene": scene.to_dict(),
        "runs": runs,
        "compare": compare_metrics(runs),
    }


def render(report: dict, path: str) -> str:
    template_path = Path(__file__).parent / "report_template.html"
    html = template_path.read_text(encoding="utf-8")
    payload = json.dumps(report, ensure_ascii=False).replace("</", "<\\/")
    html = html.replace("__FLIGHTVLA_DATA__", payload)
    path = str(path)
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(html)
    # machine-readable twin for offline re-scoring (docs/action-format.md "Run logs")
    with open(path + ".json", "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False)
    return path
