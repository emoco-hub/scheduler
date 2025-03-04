from setuptools import setup
import os
from glob import glob

package_name = "scheduler_node"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools", "schedule"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="user@example.com",
    description="Scheduler node for ROS 2 automation, storing schedules as ROS parameters.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scheduler_node = scheduler_node.scheduler_node:main",
        ],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
)
