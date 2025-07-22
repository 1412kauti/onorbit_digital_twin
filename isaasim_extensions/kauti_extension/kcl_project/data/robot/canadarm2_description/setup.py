from setuptools import find_packages, setup
import os

package_name = 'canadarm2_description'

def package_files(directory):
    paths=[]
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths        

urdf_files = package_files('urdf')
mesh_files = package_files('meshes')
launch_files = package_files('launch')
rviz_files = package_files('rviz') if os.path.exists('rviz') else []

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Add urdf, meshes, launch, and rviz files to data_files
if urdf_files:
    data_files.append(('share/' + package_name + '/urdf', urdf_files))
if mesh_files:
    data_files.append(('share/' + package_name + '/meshes', mesh_files))
if launch_files:
    data_files.append(('share/' + package_name + '/launch', launch_files))
if rviz_files:
    data_files.append(('share/' + package_name + '/rviz', rviz_files))


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kautilya',
    maintainer_email='kautilya97@gmail.com',
    description='CanadaArm2 URDF Description package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
