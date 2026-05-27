import os
from glob import glob

from setuptools import find_packages, setup

package_name = "moonmapper_autonomy"

_launch_files = glob(os.path.join("launch", "*.launch.py"))
_data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
]
if _launch_files:
    _data_files.append(
        (os.path.join("share", package_name, "launch"), _launch_files),
    )

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=_data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MoonMapper",
    maintainer_email="",
    description="Enkle autonomi-noder for MoonMapper (depth til scan, safety).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Aktiv i autonomous_exploration_full (via nav2_rtabmap_navigation):
            "depth_to_scan_node = moonmapper_autonomy.depth_to_scan_node:main",
            "safety_obstacle_node = moonmapper_autonomy.safety_obstacle_node:main",
            # Øvrige noder: arkiverte_koder/gamle_nodes/moonmapper_autonomy/
        ],
    },
)
