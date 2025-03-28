
import json
import argparse
from solver import generate_trajectory
from animate import animate_trajectory

def main():
    parser = argparse.ArgumentParser(description="Generate a trajectory between named poses")
    parser.add_argument("--from", dest="from_pose", required=True, help="Name of starting pose")
    parser.add_argument("--to", dest="to_pose", required=True, help="Name of target pose")
    parser.add_argument("--poses", default="pose_definitions.json", help="JSON file with named poses")
    parser.add_argument("--config", default="robot_config.json", help="Robot config file")
    parser.add_argument("--out", default="trajectory.json", help="Output trajectory path")
    parser.add_argument("--animate", action="store_true", help="Visualize the result")
    args = parser.parse_args()

    # Load named poses
    with open(args.poses) as f:
        pose_defs = json.load(f)

    if args.from_pose not in pose_defs or args.to_pose not in pose_defs:
        print(f"❌ Unknown pose name. Available: {list(pose_defs.keys())}")
        return

    from_pose = pose_defs[args.from_pose]
    to_pose = pose_defs[args.to_pose]

    # Load robot config
    with open(args.config) as f:
        config = json.load(f)

    # Generate trajectory
    traj = generate_trajectory([from_pose, to_pose], config)

    # Save to file
    with open(args.out, "w") as f:
        json.dump(traj, f, indent=2)

    print(f"✅ Exported trajectory: {args.from_pose} → {args.to_pose} to {args.out}")

    if args.animate:
        animate_trajectory(traj)

if __name__ == "__main__":
    main()
