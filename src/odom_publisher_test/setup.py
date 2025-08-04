from setuptools import find_packages, setup

package_name = 'odom_publisher_test'

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
    maintainer='voris',
    maintainer_email='miguel.luz@labmetro.ufsc.br',
    description='Publica mensagens de odometria para testes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_publisher_node = odom_publisher_test.odom_publisher_node:main',
        ],
    },
)
