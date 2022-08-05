from setuptools import setup

package_name = 'create_robot_movement_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Van Acker Bert',
    author_email='bva.bmkr@gmail.com',
    maintainer='Van Acker Bert',
    maintainer_email='bva.bmkr@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Getting started with the irobot create movement',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Movement_basic = create_robot_movement_py.move_simple:main',
            'Avoidance_naive = create_robot_movement_py.move_avoid_naive:main'
        ],
    },
)
