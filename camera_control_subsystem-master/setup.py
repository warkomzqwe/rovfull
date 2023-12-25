from setuptools import setup, find_packages

setup(
  name='cameracontrolsubsystem',
  version='0.2.0',
  packages=['camera_control_subsystem'],
  url='',
  license='MIT',
  author='nhasbun',
  author_email='nhasbun@gmail.com',
  description='',

  # python version requirement
  python_requires='>=3',
  install_requires=[
  'ruamel.yaml',
  'mqttudp@https://github.com/dzavalishin/mqtt_udp/archive/0a38cfdb724c7f662188153d0fb02e4ae0f6e6aa.zip#egg=mqttudp-0.5-0&subdirectory=lang/python3',
  'pymessaginglib@git+http://gitlab+deploy-token-5:kv7ktsPptL9kd1Z1PZzH@gitlab.telemcloud.cl/nhasbun/py_messaging_lib.git@release-0.0.1#egg=pymessaginglib-0.0.1',
  ],
)

