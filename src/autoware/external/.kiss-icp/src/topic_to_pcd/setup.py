from setuptools import find_packages, setup

package_name = 'topic_to_pcd'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='kimjusung2109@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'topic_to_pcd_node=topic_to_pcd.topic_to_pcd_node:main', 
            'save_pcd_map_service_node=topic_to_pcd.save_pcd_map_service:main',
        ],
    },
)
