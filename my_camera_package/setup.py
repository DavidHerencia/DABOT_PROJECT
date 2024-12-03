from setuptools import find_packages, setup

package_name = 'my_camera_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/scripts', ['scripts/pub_camera.py', 'scripts/sus_camera.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dh',
    maintainer_email='david.herencia@utec.edu.pe',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_camera = my_camera_package.pub_camera:main',  # Sin el "scripts"
            'sus_camera = my_camera_package.sus_camera:main',  # Sin el "scripts"
        ],
    },
)
