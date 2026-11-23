from glob import glob

from setuptools import find_packages, setup

package_name = "tewatashi_task"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/web", glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="suzutaro",
    maintainer_email="todo@example.com",
    description="Resident task node (YASMIN state machine) and operator panel.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "task_node = tewatashi_task.task_node:main",
        ],
    },
)
