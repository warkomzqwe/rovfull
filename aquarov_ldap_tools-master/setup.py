from setuptools import setup, find_packages

setup(
    name='aquarov_ldap_tools',
    version='1.0.0',
    packages=find_packages(where='src', exclude=["tests"]),
    package_dir={
        '': 'src',
    },
    url='',
    license='Copyright © 2023 Aquarov',
    author='nhasbun',
    author_email='nhasbun@aquarov.cl',
    description='LDAP Administration CLI tools for Aquarov dev team.',

    # python version requirement
    python_requires='>=3.7',

    install_requires=[
        'python-ldap<3.5',
        'tabulate<0.10',
    ],

    extras_require={
        'testing': [
            'pytest',
            'pytest-mock',
            'pytest-cov'
        ],
    },

    entry_points={
        'console_scripts': [
            'ldap_admin=aquarov_ldap_tools.admin_tui:main',
            'ldap_user=aquarov_ldap_tools.user_tui:main',
        ]
    },
)
