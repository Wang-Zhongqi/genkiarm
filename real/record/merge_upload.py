"""Merge the chess datasets and upload them to Hugging Face."""

import argparse
import gc
import json
import shutil
from pathlib import Path


DATASETS = Path(__file__).resolve().parent / "datasets"
REPO_ID = "Diking2000/genkiarm-chinese-chess"


def promote_dataset(temporary, output):
    """Install the validated merge, retaining the original if promotion fails."""
    backup = output.with_name(output.name + "_old")
    workspace = DATASETS.resolve()
    for path in (temporary, output, backup):
        resolved = path.resolve()
        if resolved == workspace or not resolved.is_relative_to(workspace):
            raise ValueError(f"Replacement must stay inside {workspace}: {resolved}")
    if backup.exists():
        raise FileExistsError(f"Previous backup exists; inspect it first: {backup}")
    had_original = output.exists()
    if had_original:
        output.rename(backup)
    try:
        temporary.rename(output)
    except OSError:
        if had_original:
            backup.rename(output)
        raise
    if had_original:
        # Check the final absolute deletion target immediately before deletion.
        target = backup.resolve()
        if target != workspace and target.is_relative_to(workspace) and not backup.is_symlink():
            shutil.rmtree(target)
        else:
            raise ValueError(f"Refusing to delete unexpected backup path: {target}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, default=DATASETS / "chinese_chess")
    parser.add_argument("--add", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=DATASETS / "chinese_chess")
    parser.add_argument("--repo-id", default=REPO_ID)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="Check inputs without merging or uploading")
    mode.add_argument("--no-upload", action="store_true", help="Merge locally only")
    mode.add_argument("--upload-only", action="store_true", help="Upload an already merged output")
    args = parser.parse_args()
    output = args.output.resolve()
    temporary = output.with_name(output.name + "_tmp")

    if not args.upload_only:
        sources = [args.base.resolve(), args.add.resolve()]
        if sources[0] == sources[1] or output == sources[1] or any(
            source != output and (output.is_relative_to(source) or source.is_relative_to(output))
            for source in sources
        ):
            parser.error("Output may replace --base, but must be separate from --add.")
        if output.exists() and output != sources[0]:
            parser.error(f"Output already exists: {output}. Use --upload-only to retry uploading.")
        if not output.is_relative_to(DATASETS.resolve()) or output == DATASETS.resolve():
            parser.error("Output must be a dataset directory inside real/record/datasets.")
        for path in (temporary, output.with_name(output.name + "_old")):
            if path.exists():
                parser.error(f"Temporary/backup directory exists; inspect it first: {path}")
        if any(temporary == source or source.is_relative_to(temporary) for source in sources):
            parser.error("Temporary directory must be separate from the inputs.")
        infos = [json.loads((p / "meta/info.json").read_text(encoding="utf-8")) for p in sources]
        for key in ("codebase_version", "fps", "features", "robot_type"):
            if infos[0][key] != infos[1][key]:
                parser.error(f"Datasets have different {key}; cannot merge without review.")
        for path, info in zip(sources, infos):
            print(f"Input: {path} ({info['total_episodes']} episodes)")
        expected = sum(info["total_episodes"] for info in infos)
        print(f"Output: {output} ({expected} episodes)")
        print(f"Temporary merge: {temporary}; old output will be removed after replacement.")
        print(f"Hub repository: {args.repo_id}")
        if args.dry_run:
            return

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    if args.upload_only:
        merged = LeRobotDataset(args.repo_id, root=output)
    else:
        from lerobot.datasets.dataset_tools import merge_datasets

        datasets = [
            LeRobotDataset(f"local/{path.name}", root=path)
            for path in sources
        ]
        merged = merge_datasets(datasets, output_repo_id=args.repo_id, output_dir=temporary)
        if merged.meta.total_episodes != expected:
            raise RuntimeError("Merged episode count does not match inputs; upload cancelled.")
        if merged.meta.total_frames != sum(info["total_frames"] for info in infos):
            raise RuntimeError("Merged frame count does not match inputs; upload cancelled.")
        print(f"Merged: {merged.meta.total_episodes} episodes, {merged.meta.total_frames} frames")
        # Release readers before renaming directories on Windows.
        del datasets, merged
        gc.collect()
        promote_dataset(temporary, output)
        merged = LeRobotDataset(args.repo_id, root=output)

    if not args.no_upload:
        # Creates a private repository if absent; preserves existing visibility.
        # LeRobot also publishes the dataset card and v3.0 version tag.
        merged.push_to_hub(private=True)
        print(f"Uploaded: https://huggingface.co/datasets/{args.repo_id}")
    else:
        print("Saved locally. Use --upload-only to upload this output later.")


if __name__ == "__main__":
    main()
