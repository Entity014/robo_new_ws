import os
from glob import glob
from setuptools import setup

package_name = "nong_robo"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="entity014",
    maintainer_email="phytes.narawit@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drive_node = nong_robo.drive:main",
            "state_node = nong_robo.state_selector:main",
            "detection_node = nong_robo.detection:main",
            "command_node = nong_robo.command:main",
        ],
    },
)
