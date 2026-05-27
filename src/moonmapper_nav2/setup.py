from setuptools import setup

package_name = "moonmapper_nav2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_dir={package_name: "moonmapper_nav2"},
    zip_safe=True,
    maintainer="MoonMapper Team",
    maintainer_email="user@example.com",
    description="Nav2 og RTAB-utforsking for MoonMapper",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "frontier_explorer = moonmapper_nav2.frontier_explorer:main",
            "nav2_map_ready_wait_node = moonmapper_nav2.nav2_map_ready_wait_node:main",
        ],
    },
)
