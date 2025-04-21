from setuptools import find_packages, setup

package_name = 'pcd_type_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/convert_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='k',
    maintainer_email='kimjusung2109@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'convert_node = pcd_type_converter.convert_node:main'
        ],
    },
)
