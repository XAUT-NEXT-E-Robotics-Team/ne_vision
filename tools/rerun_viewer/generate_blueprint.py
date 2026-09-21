"""生成双视图蓝图。依赖：pip install rerun-sdk==0.37.1。"""

import argparse
from pathlib import Path

import rerun.blueprint as rrb


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--app-id", default="nv", help="对应配置 rerun_debug.name")
    parser.add_argument("--root", default="/rerun_debug", help="调试实体根路径")
    parser.add_argument(
        "--output", type=Path, default=Path(__file__).with_name("nv_dual.rbl")
    )
    args = parser.parse_args()
    imu = f"{args.root.rstrip('/')}/imu"
    frame = f"{imu}/camera/frame"
    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(name="3D Scene", origin=imu, contents=[f"{imu}/**"]),
            rrb.Spatial2DView(
                name="Camera + 3D Boxes",
                origin=frame,
                contents=[f"{frame}/**", f"{imu}/detected_armor3d/**"],
            ),
            column_shares=[1, 1],
        ),
        auto_layout=False,
        auto_views=False,
    )
    blueprint.save(args.app_id, args.output)
    print(f"Saved {args.output} (application_id={args.app_id})")


if __name__ == "__main__":
    main()
