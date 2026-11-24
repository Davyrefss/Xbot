from setuptools import find_packages, setup

package_name = 'xbot_test_python'

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
    maintainer='davyrefss',
    maintainer_email='verity3261447913@163.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = xbot_test_python.talker:main',
            'listener = xbot_test_python.listener:main',
            'add_server = xbot_test_python.add_server:main',
            'add_client = xbot_test_python.add_client:main',
            'param_demo = xbot_test_python.param_demo:main',
            'fibonacci_server = xbot_test_python.fibonacci_server:main',
            'fibonacci_client = xbot_test_python.fibonacci_client:main',
            'executor_demo = xbot_test_python.executor_demo:main',
        ],
    },
)
