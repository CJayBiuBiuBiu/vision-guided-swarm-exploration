"""Prepare a fresh ROS/Gazebo checkout from the archived baseline and project patches."""

import argparse
import subprocess
import tempfile
from pathlib import Path


BASELINE_REPOSITORY = "https://github.com/CJayBiuBiuBiu/turtlebot3_ddpg_collision_avoidance.git"
BASELINE_REVISION = "39ee535f99ba6e061b1d7db4658a03d009a61dcc"
PATCH_DIRECTORY = Path(__file__).resolve().parents[1] / "ros_gazebo" / "patches"


def run_git(directory, *arguments):
    subprocess.run(["git", "-C", str(directory), *arguments], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path, help="a new directory in the ROS workspace")
    args = parser.parse_args()
    destination = args.destination.expanduser().absolute()
    if destination.exists() or destination.is_symlink():
        parser.error(f"destination already exists: {destination}")

    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".vision-ros-", dir=destination.parent) as directory:
        checkout = Path(directory)
        run_git(checkout, "init", "--quiet")
        run_git(checkout, "fetch", "--quiet", "--no-tags", "--depth=1",
                BASELINE_REPOSITORY, BASELINE_REVISION)
        run_git(checkout, "checkout", "--quiet", "--detach", BASELINE_REVISION)

        for patch in sorted(PATCH_DIRECTORY.glob("*.patch")):
            run_git(checkout, "apply", "--index", str(patch))
            run_git(checkout, "-c", "user.name=Vision-Guided Swarm Exploration",
                    "-c", "user.email=ros-gazebo-build@localhost",
                    "commit", "--quiet", "--no-gpg-sign", "-m", f"Apply {patch.stem}")

        # Reserve the name before replacing the empty directory. A concurrent
        # run reaching this point first keeps its checkout.
        destination.mkdir()
        try:
            checkout.replace(destination)
        except OSError:
            destination.rmdir()
            raise

    print(f"ROS/Gazebo checkout: {destination}", flush=True)
    run_git(destination, "show", "-s", "--format=Revision: %H%nTree: %T", "HEAD")


if __name__ == "__main__":
    main()
