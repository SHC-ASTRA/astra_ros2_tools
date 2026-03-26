from setuptools import find_packages, setup

package_name = "feedback_generator"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="David",
    maintainer_email="ds0196@uah.edu",
    description="TODO: Package description",
    license="AGPL-3.0-only",
    entry_points={
        "console_scripts": [
            "feedback_generator_core = feedback_generator.feedback_node_core:main",
            "feedback_generator_arm = feedback_generator.feedback_node_arm:main",
        ],
    },
)
