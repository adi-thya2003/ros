from setuptools import find_packages, setup

package_name = 'turtsim'

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
    maintainer='adithya',
    maintainer_email='adithya@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hear = turtsim.hearsim:main',
            'rec = turtsim.vorec:main',
            'play= turtsim.play:main',
            'cont = turtsim.controller:main',
            'turc = turtsim.turtlecont:main',
            'bot =turtsim.turtlebot:main',
        ],
    },
)
