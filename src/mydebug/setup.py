from setuptools import setup

package_name = 'mydebug'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='henhen.luo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recv_single = mydebug.recv_single:main',
            'send_single = mydebug.send_single:main',
            'recv_triple = mydebug.recv_triple:main',
            'send_triple = mydebug.send_triple:main',
        ],
    },
)
