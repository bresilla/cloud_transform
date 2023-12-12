from glob import glob
from setuptools import setup
import os

package_name = 'cloud_transform'

data_files = []
#resources
data_files.append(('share/' + package_name + '/resource', glob('resource/*')))
data_files.append(('share/' + package_name, ['package.xml']))

#launch files
data_files.append((os.path.join('share', package_name), glob('launch/*')))
#params

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bresilla',
    maintainer_email='trim.bresilla@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cloud_transform = cloud_transform.cloud_transform:main',
            'cloud_joiner = cloud_transform.cloud_joiner:main',
        ],
        'launch.frontend.launch_extension': [
            'launch_ros = launch_ros'
        ]
    },
)
