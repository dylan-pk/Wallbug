from setuptools import setup

package_name = 'wallbot_bot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Connor Williams',
    maintainer_email='cajwilliams13@example.com',
    description='URDF description package for Wallbot V3',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            # add Python nodes if you have any, otherwise leave empty
        ],
    },
)
