from setuptools import find_packages, setup

package_name = 'assistbot_base'

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
    maintainer='sayed',
    maintainer_email='elsayed.elsheikh97@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'assistbot_client = assistbot_base.assistbot_client:main',
            'assistbot_server = assistbot_base.assistbot_server:main',
            'assistbot_base = assistbot_base.assistbot_base:main'
        ],
    },
)
