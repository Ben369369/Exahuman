#!/usr/bin/env python3
r"""Create a filtered LeRobot v3 dataset without changing the source dataset.

Usage
-----
Keep two cameras and remove robot state (visual-only AM-ACT dataset):

    python -m lerobot.scripts.create_alohamini_visual_only_dataset \
      --source /path/to/source_dataset \
      --target /path/to/copied_dataset \
      --keep-camera forward \
      --keep-camera wrist_right \
      --drop-state


Copy files instead of using hard links:

    python -m lerobot.scripts.create_alohamini_visual_only_dataset \
      --source /path/to/source_dataset \
      --target /path/to/copied_dataset \
      --mode copy

Defaults are safe: keep every camera, keep observation.state, and use hard
links when possible with automatic copy fallback. The target must not exist.

For visual-only training, AM-ACT supports a dataset without observation.state.
Most other policies require state; check the selected policy before using
--drop-state.
"""

from __future__ import annotations

import argparse
import errno
import json
import os
import shutil
from dataclasses import dataclass
from pathlib import Path

import pyarrow.parquet as pq

OBS_STATE = "observation.state"
MEDIA_DTYPES = {"image", "video"}
PROTECTED_FEATURES = {
    "action",
    "timestamp",
    "frame_index",
    "episode_index",
    "index",
    "task_index",
}


@dataclass
class TransferCounts:
    linked: int = 0
    copied: int = 0
    rewritten: int = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a filtered LeRobot v3 dataset view.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--source", type=Path, required=True, help="Source LeRobot dataset directory.")
    parser.add_argument("--target", type=Path, required=True, help="New output dataset directory.")

    camera_group = parser.add_mutually_exclusive_group()
    camera_group.add_argument(
        "--keep-camera",
        action="append",
        default=[],
        metavar="NAME",
        help="Keep only this camera. Repeat for multiple cameras.",
    )
    camera_group.add_argument(
        "--drop-camera",
        action="append",
        default=[],
        metavar="NAME",
        help="Remove this camera. Repeat for multiple cameras.",
    )
    parser.add_argument(
        "--drop-state",
        action="store_true",
        help="Remove observation.state. State is kept by default.",
    )
    parser.add_argument(
        "--drop-feature",
        action="append",
        default=[],
        metavar="FEATURE",
        help="Remove another observation feature. Repeat for multiple features.",
    )
    parser.add_argument(
        "--mode",
        choices=("auto", "hardlink", "copy"),
        default="auto",
        help="File transfer mode. auto tries hard links and falls back to copies.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the planned feature selection without creating the target.",
    )
    return parser.parse_args()


def load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as file:
        return json.load(file)


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=4) + "\n", encoding="utf-8")


def is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
    except ValueError:
        return False
    return True


def transfer_file(source: Path, target: Path, mode: str, counts: TransferCounts) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    if source.is_symlink():
        target.symlink_to(os.readlink(source))
        counts.linked += 1
        return

    if mode in {"auto", "hardlink"}:
        try:
            os.link(source, target)
            counts.linked += 1
            return
        except OSError as error:
            can_fallback = error.errno in {
                errno.EXDEV,
                errno.EPERM,
                errno.EACCES,
                errno.EMLINK,
                errno.ENOTSUP,
            }
            if mode == "hardlink" or not can_fallback:
                raise

    shutil.copy2(source, target)
    counts.copied += 1


def transfer_tree(source: Path, target: Path, mode: str, counts: TransferCounts) -> None:
    target.mkdir(parents=True, exist_ok=True)
    for path in source.rglob("*"):
        destination = target / path.relative_to(source)
        if path.is_dir():
            destination.mkdir(parents=True, exist_ok=True)
        elif path.is_file() or path.is_symlink():
            transfer_file(path, destination, mode, counts)


def rewrite_parquet(
    source: Path,
    target: Path,
    dropped_features: set[str],
    *,
    episode_metadata: bool,
    mode: str,
    counts: TransferCounts,
) -> None:
    table = pq.read_table(source)

    def should_drop(column: str) -> bool:
        for feature in dropped_features:
            if episode_metadata:
                prefixes = (
                    f"videos/{feature}/",
                    f"images/{feature}/",
                    f"stats/{feature}/",
                )
                if column.startswith(prefixes):
                    return True
            if column == feature or column.startswith(f"{feature}."):
                return True
        return False

    keep_columns = [column for column in table.column_names if not should_drop(column)]
    if keep_columns == table.column_names:
        transfer_file(source, target, mode, counts)
        return

    target.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(table.select(keep_columns), target)
    counts.rewritten += 1


def resolve_camera_names(requested: list[str], camera_features: list[str]) -> set[str]:
    resolved: set[str] = set()
    for name in requested:
        if name in camera_features:
            resolved.add(name)
            continue
        matches = [feature for feature in camera_features if feature.rsplit(".", 1)[-1] == name]
        if len(matches) == 1:
            resolved.add(matches[0])
        elif not matches:
            raise ValueError(
                f"Unknown camera {name!r}. Available cameras: {', '.join(camera_features) or '(none)'}"
            )
        else:
            raise ValueError(f"Ambiguous camera name {name!r}; use the full feature name.")
    return resolved


def select_features(
    info: dict,
    keep_cameras: list[str],
    drop_cameras: list[str],
    drop_state: bool,
    extra_drops: list[str],
) -> tuple[list[str], set[str]]:
    features = info.get("features")
    if not isinstance(features, dict):
        raise ValueError("meta/info.json does not contain a features mapping.")

    camera_features = [
        name
        for name, spec in features.items()
        if isinstance(spec, dict) and spec.get("dtype") in MEDIA_DTYPES
    ]
    selected_cameras = set(camera_features)
    if keep_cameras:
        selected_cameras = resolve_camera_names(keep_cameras, camera_features)
    elif drop_cameras:
        selected_cameras -= resolve_camera_names(drop_cameras, camera_features)

    dropped = set(camera_features) - selected_cameras
    if drop_state and OBS_STATE in features:
        dropped.add(OBS_STATE)

    for feature in extra_drops:
        if feature not in features:
            raise ValueError(f"Unknown feature {feature!r}.")
        if feature in PROTECTED_FEATURES or not feature.startswith("observation."):
            raise ValueError(f"Refusing to remove required feature {feature!r}.")
        dropped.add(feature)

    kept_cameras = [feature for feature in camera_features if feature not in dropped]
    if OBS_STATE in dropped and not kept_cameras:
        raise ValueError("A visual-only dataset must keep at least one camera.")
    return kept_cameras, dropped


def copy_metadata(
    source: Path,
    target: Path,
    info: dict,
    dropped_features: set[str],
    mode: str,
    counts: TransferCounts,
) -> None:
    filtered_info = dict(info)
    filtered_info["features"] = {
        name: spec for name, spec in info["features"].items() if name not in dropped_features
    }
    write_json(target / "meta/info.json", filtered_info)
    counts.rewritten += 1

    stats_path = source / "meta/stats.json"
    if stats_path.is_file():
        stats = load_json(stats_path)
        filtered_stats = {name: value for name, value in stats.items() if name not in dropped_features}
        write_json(target / "meta/stats.json", filtered_stats)
        counts.rewritten += 1

    meta_root = source / "meta"
    for path in meta_root.rglob("*"):
        relative = path.relative_to(meta_root)
        if relative in {Path("info.json"), Path("stats.json")}:
            continue
        destination = target / "meta" / relative
        if path.is_dir():
            destination.mkdir(parents=True, exist_ok=True)
        elif relative.parts and relative.parts[0] == "episodes" and path.suffix == ".parquet":
            rewrite_parquet(
                path,
                destination,
                dropped_features,
                episode_metadata=True,
                mode=mode,
                counts=counts,
            )
        elif path.is_file() or path.is_symlink():
            transfer_file(path, destination, mode, counts)


def copy_data(
    source: Path,
    target: Path,
    dropped_features: set[str],
    mode: str,
    counts: TransferCounts,
) -> None:
    data_root = source / "data"
    if not data_root.is_dir():
        raise FileNotFoundError(f"Missing data directory: {data_root}")
    for path in data_root.rglob("*"):
        destination = target / "data" / path.relative_to(data_root)
        if path.is_dir():
            destination.mkdir(parents=True, exist_ok=True)
        elif path.suffix == ".parquet":
            rewrite_parquet(
                path,
                destination,
                dropped_features,
                episode_metadata=False,
                mode=mode,
                counts=counts,
            )
        elif path.is_file() or path.is_symlink():
            transfer_file(path, destination, mode, counts)


def copy_media(
    source: Path,
    target: Path,
    info: dict,
    kept_cameras: list[str],
    mode: str,
    counts: TransferCounts,
) -> None:
    for feature in kept_cameras:
        dtype = info["features"][feature]["dtype"]
        media_root = "videos" if dtype == "video" else "images"
        source_dir = source / media_root / feature
        if not source_dir.is_dir():
            raise FileNotFoundError(f"Missing media directory for {feature}: {source_dir}")
        transfer_tree(source_dir, target / media_root / feature, mode, counts)


def copy_other_top_level_files(
    source: Path,
    target: Path,
    mode: str,
    counts: TransferCounts,
) -> None:
    handled = {"meta", "data", "videos", "images"}
    for path in source.iterdir():
        if path.name in handled:
            continue
        destination = target / path.name
        if path.is_dir():
            transfer_tree(path, destination, mode, counts)
        elif path.is_file() or path.is_symlink():
            transfer_file(path, destination, mode, counts)


def main() -> None:
    args = parse_args()
    source = args.source.expanduser().resolve()
    target = args.target.expanduser().resolve()

    info_path = source / "meta/info.json"
    if not info_path.is_file():
        raise FileNotFoundError(f"Not a LeRobot v3 dataset: {source}")
    if target.exists():
        raise FileExistsError(f"Target already exists; refusing to modify it: {target}")
    if source == target or is_relative_to(target, source):
        raise ValueError("Target must not be the source directory or a child of it.")

    info = load_json(info_path)
    kept_cameras, dropped_features = select_features(
        info,
        keep_cameras=args.keep_camera,
        drop_cameras=args.drop_camera,
        drop_state=args.drop_state,
        extra_drops=args.drop_feature,
    )

    print(f"Source: {source}")
    print(f"Target: {target}")
    print(f"Kept cameras: {', '.join(kept_cameras) or '(none)'}")
    print(f"State: {'removed' if OBS_STATE in dropped_features else 'kept'}")
    print(f"Removed features: {', '.join(sorted(dropped_features)) or '(none)'}")
    print(f"Transfer mode: {args.mode}")
    if args.dry_run:
        print("Dry run: no files created.")
        return

    counts = TransferCounts()
    target.mkdir(parents=True)
    try:
        copy_metadata(source, target, info, dropped_features, args.mode, counts)
        copy_data(source, target, dropped_features, args.mode, counts)
        copy_media(source, target, info, kept_cameras, args.mode, counts)
        copy_other_top_level_files(source, target, args.mode, counts)
    except Exception:
        shutil.rmtree(target)
        raise

    print(f"Created dataset: {target}")
    print(
        f"Files: {counts.linked} hard-linked, {counts.copied} copied, "
        f"{counts.rewritten} metadata/parquet files rewritten."
    )


if __name__ == "__main__":
    main()
