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

"""Extract a compact regression bag from a large rosbag2 bag."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import zstandard
from rosbag2_py import ConverterOptions
from rosbag2_py import SequentialReader
from rosbag2_py import SequentialWriter
from rosbag2_py import StorageOptions
from rosbag2_py import TopicMetadata


TOPICS = [
    "/cabot/cmd_vel",
    "/cabot/controller_status_left",
    "/cabot/controller_status_right",
    "/cabot/control_message_left",
    "/cabot/control_message_right",
    "/cabot/odom_raw",
]
ZSTD_MAGIC = b"\x28\xb5\x2f\xfd"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-bag",
        required=True,
        help="Path to source rosbag directory (contains metadata.yaml)",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        help="Output bag directory",
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

    source_bag = Path(args.source_bag)
    output_dir = Path(args.output_dir)
    end_ns = args.start_ns + int(args.duration_sec * 1e9)

    if output_dir.exists():
        shutil.rmtree(output_dir)

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(source_bag), storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    topic_map = {topic.name: topic for topic in reader.get_all_topics_and_types()}
    selected_topics = [name for name in TOPICS if name in topic_map]
    if not selected_topics:
        raise RuntimeError("None of the required topics were found in source bag.")

    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=str(output_dir), storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    zstd_decompressor = zstandard.ZstdDecompressor()

    for topic_name in selected_topics:
        meta = topic_map[topic_name]
        writer.create_topic(
            TopicMetadata(
                name=meta.name,
                type=meta.type,
                serialization_format=meta.serialization_format,
                offered_qos_profiles=meta.offered_qos_profiles,
            )
        )

    message_count = 0
    selected_set = set(selected_topics)
    while reader.has_next():
        topic_name, raw_data, timestamp = reader.read_next()
        if topic_name not in selected_set:
            continue
        if timestamp < args.start_ns or timestamp > end_ns:
            continue
        if raw_data.startswith(ZSTD_MAGIC):
            raw_data = zstd_decompressor.decompress(raw_data)
        writer.write(topic_name, raw_data, timestamp)
        message_count += 1

    print(f"Wrote {message_count} messages to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
