import os
import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'a4vai'
controller = 'a4vai/controller'
path_following = 'a4vai/path_following'
path_planning = 'a4vai/path_planning'
collision_avoidance = 'a4vai/collision_avoidance'
map_queue = 'a4vai/map_queue'

share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, controller, path_following, path_planning, collision_avoidance, map_queue],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        # (share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        # (share_dir + '/param', glob.glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='SeJun, Kim',
    author_email='kimsejoon@icloud.com',
    maintainer='SeJun',
    maintainer_email='kimsejoon@icloud.com',
    description='A4VAI 2nd Study Group Algorithm Test Simulator',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = a4vai.controller.main:main',
            'JBNU_module = a4vai.collision_avoidance.main:main',
            'path_following_gpr = a4vai.path_following.pf_gpr_module:main',
            'path_following_att = a4vai.path_following.pf_attitude_cmd_module:main',
            'path_following_guid = a4vai.path_following.pf_guid_param_module:main',
            'deep_sac_module = a4vai.path_planning.main:main',
            'map_queue = a4vai.map_queue.map_queue:main',
            'LOSCA_module = a4vai.collision_avoidance.main:main',
        ],
    },
)
