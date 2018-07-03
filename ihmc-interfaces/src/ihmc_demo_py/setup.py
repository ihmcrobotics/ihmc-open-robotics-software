from setuptools import setup

package_name = 'ihmc_demo_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    py_modules=[
        'rcd_pub',
        'rcd_sub'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    author='Duncan Calvert',
    author_email='dcalvert@ihmc.us',
    maintainer='Duncan Calvert',
    maintainer_email='dcalvert@ihmc.us',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Example nodes for interacting/testing ihmc software.',
    license='Apache License, Version 2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rcd_sub = rcd_sub:main',
            'rcd_pub = rcd_pub:main'
        ],
    },
)