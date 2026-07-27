#!/usr/bin/env python3
"""Regenerates teams.html's Pool 1/Pool 2 tables from a GridMind pools config.

Reads team NAMES only from the pools JSON (e.g. gridmind-referee/data/pools_6teams.json)
-- never the MAC addresses paired with them there, since those are gridmind's own
internal board-registration detail and have no place on the public website.

Usage:
    python3 generate_teams_page.py --pools /path/to/pools_6teams.json --output /path/to/teams.html
"""
import argparse
import json
import re
from pathlib import Path

POOL_TABLE_TEMPLATE = """        <h3>Pool {pool_num}</h3>
        <table style="width:100%">
            <tr>
              <th>Team Name</th>
              <th>Members</th>
              <th>Intern</th>
            </tr>
{rows}
          </table>
"""

ROW_TEMPLATE = """            <tr>
              <td>{name}</td>
              <td>&nbsp;</td>
              <td>&nbsp;</td>
            </tr>"""


def build_pool_table(pool_num, team_names):
    rows = "\n".join(ROW_TEMPLATE.format(name=name) for name in team_names)
    return POOL_TABLE_TEMPLATE.format(pool_num=pool_num, rows=rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pools", required=True, help="Path to gridmind pools_*.json")
    ap.add_argument("--output", required=True, help="Path to teams.html to update")
    args = ap.parse_args()

    config = json.loads(Path(args.pools).read_text())
    pool1_names = [name for name, _mac in config["pool1_teams"]]
    pool2_names = [name for name, _mac in config["pool2_teams"]]

    new_tables = build_pool_table(1, pool1_names) + "\n" + build_pool_table(2, pool2_names)

    html = Path(args.output).read_text()
    updated = re.sub(
        r"        <h3>Pool 1</h3>.*?</table>\n\n        <h3>Pool 2</h3>.*?</table>\n",
        new_tables,
        html,
        flags=re.DOTALL,
    )
    if updated == html:
        raise SystemExit("Pool 1/Pool 2 tables not found in output file -- refusing to write unchanged content")
    Path(args.output).write_text(updated)
    print(f"Updated {args.output}: Pool 1 ({len(pool1_names)} teams), Pool 2 ({len(pool2_names)} teams)")


if __name__ == "__main__":
    main()
