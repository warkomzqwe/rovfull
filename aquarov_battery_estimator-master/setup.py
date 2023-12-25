from setuptools import setup, find_namespace_packages

setup(
    name='aquarov_battery_estimator',
    version='0.1.0',
    packages=find_namespace_packages(include=['aquarov_battery_estimator',
                                              'aquarov_battery_estimator.*']),
    url='',
    license='Copyright © 2020 Aquarov',
    author='nhasbun',
    author_email='nhasbun@gmail.com',
    description='',

    # python version requirement
    python_requires='>=3.6',
    install_requires=[
        'event-bus==1.0.2',
        'pymessaginglib@git+http://gitlab+deploy-token-5:kv7ktsPptL9kd1Z1PZzH@gitlab.telemcloud.cl/nhasbun/py_messaging_lib.git',
        'rov-logger@git+http://gitlab+deploy-token-2:LtRBjuvD9CiJyawrsvsr@gitlab.telemcloud.cl/aquarovdevelopment/rov-logger.git@master',
    ],

    entry_points={
          'console_scripts': [
              'battery_estimator_service=aquarov_battery_estimator.battery_estimator:main',
          ]
        },
)
