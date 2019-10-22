from setuptools import setup

package_name = 'robug'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    author='Felar',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: None',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Description TBD',
    license='None',
    entry_points={
        'console_scripts': [
            'robug_exec ='
            ' robug.robug_main:main'
        ],
    },
)
