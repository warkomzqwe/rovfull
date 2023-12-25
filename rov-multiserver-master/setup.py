#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Setup script for rov-multiserver."""
from setuptools import find_packages, setup

project = "rov-multiserver"
version = "0.1.0"

setup(
    name=project,
    version=version,
    description="ROV system control and video server",
    author="Jorge Barría",
    author_email="jbarria@aquarov.cl",
    url="http://gitlab.telemcloud.cl/aquarovdevelopment/rov-multiserver",
    license="Proprietary",
    packages=find_packages(where='src', exclude=["tests"]),
    package_dir={
        '': 'src',
    },
    include_package_data=True,
    zip_safe=False,
    install_requires=[
        'click',
        'pyserial',
        'pymavlink',
        'aiojobs',
        'ruamel.yaml<=0.16.10',
        'python-singleton',
        'async_timeout',
        'minimalmodbus',
        'rov-logger'
        '@git+http://gitlab+deploy-token-2:LtRBjuvD9CiJyawrsvsr'
        '@gitlab.telemcloud.cl/aquarovdevelopment/rov-logger.git'
        '@master',
        'rov-event-bus'
        '@git+http://gitlab+deploy-token-3:jFsjFtbmYjsjJN4SejMx'
        '@gitlab.telemcloud.cl/aquarovdevelopment/rov-event-bus.git'
        '@master',
        'pymessaginglib'
        '@git+http://gitlab+deploy-token-5:kv7ktsPptL9kd1Z1PZzH'
        '@gitlab.telemcloud.cl/nhasbun/py_messaging_lib.git'
        '@master'
    ],
    setup_requires=[
        'pytest-runner'
    ],
    dependency_links=[
    ],
    entry_points={
        'console_scripts': [
            'rov-server=rov_multiserver.main:main',
            'rov-ctl=rov_multiserver.rov_remote_control:main',
            'psmd=rov_multiserver.psm_muxer_daemon:main',
            'rov-psmd=rov_multiserver.psm_muxer_daemon:main',
            'rov-psd=uservices.power_source_detector_daemon:main',
            'rov-hourmeterd=uservices.hourmeter_daemon:main'
        ]
    },
    extras_require={
    },
    tests_require=[
        'pytest',
        'pytest-cov'
    ]
)
