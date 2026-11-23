from glob import glob

from setuptools import find_packages, setup

package_name = "tewatashi_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "requirements.txt"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="suzutaro",
    maintainer_email="todo@example.com",
    description="Perception service nodes (SAM3, ...).",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "sam3_server = tewatashi_perception.sam3_server:main",
        ],
    },
)
