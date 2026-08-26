import os
from glob import glob

from setuptools import find_packages, setup


package_name = "p03_rviz_route_planner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="todo",
    maintainer_email="todo@todo.todo",
    description="RViz hand-drawn route planner for ROS 2.",
    license="MIT",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "p03_rviz_route_planner = p03_rviz_route_planner.rviz_route_planner:main",
            "astar_global_planner = p03_rviz_route_planner.astar_global_planner:main",
            "rrt_global_planner = p03_rviz_route_planner.rrt_global_planner:main",
        ],
    },
)
