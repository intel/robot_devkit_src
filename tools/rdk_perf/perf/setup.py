# Copyright (c) 2019 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import glob
import os
from setuptools import find_packages
from setuptools import setup

package_name = 'rdk_perf'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=[
        'launch',
        'setuptools',
    ],
    zip_safe=True,
    author='Xiaojun Huang',
    author_email='xiaojun.huang@intel.com',
    maintainer='Xiaojun Huang',
    maintainer_email='xiaojun.huang@intel.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Python nodes for performance measurement.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delay = scripts.delay:main',
            'hz = scripts.hz:main',
        ],
    },
)
