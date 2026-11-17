from setuptools import find_packages, setup

setup(
    name="rm_odometry_phase3",
    version="0.2.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/rm_odometry_phase3"]),
        ("share/rm_odometry_phase3", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Muhammad Qaisar Ali",
    maintainer_email="student@example.invalid",
    description="RoboMaster Phase 3 fixed-gain visual odometry correction",
    license="MIT",
    entry_points={
        "console_scripts": ["experiment = rm_odometry_phase3.experiment:main"]
    },
)
