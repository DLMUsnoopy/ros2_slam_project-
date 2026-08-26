import os
from glob import glob

from setuptools import find_packages, setup


package_name = "p03_frontier_exploration"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="todo",
    maintainer_email="todo@todo.todo",
    description="Frontier-based exploration goal selector for ROS 2.",
    license="MIT",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "frontier_explorer = p03_frontier_exploration.frontier_explorer:main",
        ],
    },
)
