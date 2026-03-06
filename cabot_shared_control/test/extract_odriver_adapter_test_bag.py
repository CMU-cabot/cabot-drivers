#!/usr/bin/env python3

# Copyright (c) 2026  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Extract a compact regression bag from a large rosbag2 sqlite DB."""

from __future__ import annotations

import argparse
import sqlite3
from pathlib import Path


TOPICS = [
    "/cabot/cmd_vel",
    "/cabot/controller_status_left",
    "/cabot/controller_status_right",
    "/cabot/control_message_left",
    "/cabot/control_message_right",
]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-db", required=True, help="Path to source ros2_topics_0.db3")
    parser.add_argument(
        "--output-dir",
        required=True,
        help="Output bag directory (contains *_0.db3 and metadata.yaml)",
    )
    parser.add_argument(
        "--output-name",
        default="odriver_adapter_reference",
        help="Base name for output db3 file",
    )
    parser.add_argument(
        "--start-ns",
        type=int,
        required=True,
        help="Start timestamp in nanoseconds",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=120.0,
        help="Extraction duration in seconds",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    source_db = Path(args.source_db)
    output_dir = Path(args.output_dir)
    output_db = output_dir / f"{args.output_name}_0.db3"
    end_ns = args.start_ns + int(args.duration_sec * 1e9)

    output_dir.mkdir(parents=True, exist_ok=True)
    for path in output_dir.glob("*"):
        path.unlink()

    src = sqlite3.connect(str(source_db))
    src.row_factory = sqlite3.Row
    dst = sqlite3.connect(str(output_db))

    schema_sql = [
        "CREATE TABLE schema(schema_version INTEGER PRIMARY KEY,ros_distro TEXT NOT NULL)",
        "CREATE TABLE metadata(id INTEGER PRIMARY KEY,metadata_version INTEGER NOT NULL,metadata TEXT NOT NULL)",
        "CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT NOT NULL,type TEXT NOT NULL,serialization_format TEXT NOT NULL,offered_qos_profiles TEXT NOT NULL)",
        "CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER NOT NULL,timestamp INTEGER NOT NULL,data BLOB NOT NULL)",
        "CREATE INDEX timestamp_idx ON messages (timestamp ASC)",
    ]
    for sql in schema_sql:
        dst.execute(sql)

    for row in src.execute("SELECT * FROM schema"):
        dst.execute("INSERT INTO schema(schema_version, ros_distro) VALUES (?, ?)", tuple(row))
    for row in src.execute("SELECT * FROM metadata"):
        dst.execute("INSERT INTO metadata(id, metadata_version, metadata) VALUES (?, ?, ?)", tuple(row))

    topic_placeholders = ",".join("?" for _ in TOPICS)
    selected_topics = list(
        src.execute(
            (
                "SELECT id,name,type,serialization_format,offered_qos_profiles "
                f"FROM topics WHERE name IN ({topic_placeholders}) ORDER BY id"
            ),
            TOPICS,
        )
    )

    if not selected_topics:
        raise RuntimeError("None of the required topics were found in source DB.")

    old_to_new_topic_id = {}
    for new_id, row in enumerate(selected_topics, start=1):
        old_to_new_topic_id[row["id"]] = new_id
        dst.execute(
            (
                "INSERT INTO topics(id,name,type,serialization_format,offered_qos_profiles) "
                "VALUES (?,?,?,?,?)"
            ),
            (
                new_id,
                row["name"],
                row["type"],
                row["serialization_format"],
                row["offered_qos_profiles"],
            ),
        )

    old_ids = tuple(old_to_new_topic_id.keys())
    id_placeholders = ",".join("?" for _ in old_ids)
    params = [*old_ids, args.start_ns, end_ns]
    rows = src.execute(
        (
            "SELECT topic_id, timestamp, data FROM messages "
            f"WHERE topic_id IN ({id_placeholders}) "
            "AND timestamp BETWEEN ? AND ? "
            "ORDER BY timestamp ASC"
        ),
        params,
    )

    message_count = 0
    for message_id, row in enumerate(rows, start=1):
        dst.execute(
            "INSERT INTO messages(id,topic_id,timestamp,data) VALUES (?,?,?,?)",
            (
                message_id,
                old_to_new_topic_id[row["topic_id"]],
                row["timestamp"],
                row["data"],
            ),
        )
        message_count = message_id

    dst.commit()
    src.close()
    dst.close()

    print(f"Wrote {message_count} messages to {output_db}")
    print(
        "Run: ros2 bag reindex "
        f"{output_dir} sqlite3 && ros2 bag info {output_dir}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
