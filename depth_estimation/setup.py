from setuptools import find_packages, setup

package_name = 'depth_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lenin.chavez',
    maintainer_email='102402199+Hyp3Boy@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_node = depth_estimation.bot_image:main',
            'depth_node = depth_estimation.depth_node:main',
            'nav_node = depth_estimation.depth_nav:main',
        ],
    },
)
