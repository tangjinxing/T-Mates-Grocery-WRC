import os
from glob import glob

from setuptools import find_packages, setup

package_name = "retail_nav_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (
            os.path.join("share", package_name, "config", "map_packages", "example_retail"),
            glob("config/map_packages/example_retail/*"),
        ),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="TianJi",
    maintainer_email="dev@tianji.local",
    description="Unified mapping/navigation facade for retail service competition.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "retail_nav_bridge = retail_nav_bridge.bridge_node:main",
            "retail_nav_http_gateway = retail_nav_bridge.http_gateway:main",
        ],
    },
)
