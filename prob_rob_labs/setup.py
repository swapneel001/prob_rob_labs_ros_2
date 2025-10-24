from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'prob_rob_labs'

launch_files = glob('launch/*_launch.py') + \
    glob('launch/*_launch.xml') + \
    glob('launch/*.yaml') + \
    glob('launch/*.yml')

data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'launch'), launch_files),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ilija Hadzic',
    maintainer_email='ih2435@columbia.edu',
    description='Prob Rob Labs',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_error_plotter = ekf_error_plotter.ekf_error_plotter:main',
            'landmark_positioner = landmark_positioner.landmark_positioner:main',
            'landmark_axis_identifier = landmark_axis_identifier.landmark_axis_identifier:main',
            'ekf_odom = ekf_odom.ekf_odom:main',
            'ground_truth_publisher = ground_truth_publisher.ground_truth_publisher:main',
            'move_through_door_bayesian = move_through_door_bayesian.move_through_door_bayesian:main',
            'bayesian_probability_calculator = bayesian_probability_calculator.bayesian_probability_calculator:main',
            'move_through_door = move_through_door.move_through_door:main',
            'image_mean_feature_x = image_mean_feature_x.image_mean_feature_x:main',
            'flaky_door_opener = flaky_door_opener.flaky_door_opener:main',
        ],
    }
)
