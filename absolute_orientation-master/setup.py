from setuptools import setup, find_namespace_packages

setup(
  name='absolute_orientation',
  version='0.2.0',
  packages=find_namespace_packages(include=['absolute_orientation',
    'absolute_orientation.*']),
  url='',
  license='MIT',
  author='nhasbun',
  author_email='nhasbun@gmail.com',
  description='',

  # python version requirement
  python_requires='>=3',
  install_requires=[
  'bitarray==1.9.2',
  'numpy<2',
  'pymata-aio',
  'rov-logger@git+http://gitlab+deploy-token-2:LtRBjuvD9CiJyawrsvsr@gitlab.telemcloud.cl/aquarovdevelopment/rov-logger.git',
  'ruamel.yaml'],
)

