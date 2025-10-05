from setuptools import find_packages, setup
import os
from glob import glob

package_name = "elkapod_controller_gui"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name), glob(os.path.join(package_name, '*.png')))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Piotr Patek",
    maintainer_email="piotrpatek17@gmail.com",
    description="Elkapod controller GUI",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "elkapod_controller_gui = elkapod_controller_gui.elkapod_gui:main"
        ],
    },
)
