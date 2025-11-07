from setuptools import setup
from glob import glob
import os

package_name = 'turtlesim_pde4430'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='PDE4430 TurtleSim exercises (ROS 2 Jazzy)',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Task 2 basics
            'straight_move   = turtlesim_pde4430.straight_move:main',
            'circular_move   = turtlesim_pde4430.circular_move:main',
            'figure_eight    = turtlesim_pde4430.figure_eight:main',
            'user_input      = turtlesim_pde4430.user_input:main',
            'goto_goal       = turtlesim_pde4430.goto_goal:main',
            'double_sweep_roomba = turtlesim_pde4430.double_sweep_roomba:main',
            'full_map_cleaner = turtlesim_pde4430.full_map_cleaner:main',


            # Coverage / multi-agent
            'multi_cleaner   = turtlesim_pde4430.multi_cleaner:main',
            'square_patrol   = turtlesim_pde4430.square_patrol:main',
            'roomba_cleaner  = turtlesim_pde4430.roomba_cleaner:main',
            'fast_lawnmower  = turtlesim_pde4430.fast_lawnmower:main',

            # Task 3 advanced
            'goto_goal_adv   = turtlesim_pde4430.goto_goal_adv:main',
            'goto_goal_loop  = turtlesim_pde4430.goto_goal_loop:main',
        ],
    },
)
