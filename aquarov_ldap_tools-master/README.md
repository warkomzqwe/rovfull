# LDAP Tools

LDAP Administration CLI tools for Aquarov dev team.

## Installation

* Python 3.7+ compatible

```bash
cd $HOME
git clone http://gitlab+deploy-token-45:XZ32bYWqFSzMSRGMvaRu@gitlab.telemcloud.cl/aquarovdevelopment/aquarov_ldap_tools && \
cd aquarov_ldap_tools && python3 -m venv venv && . ./venv/bin/activate && \
pip install -U pip setuptools wheel && \
pip install -e .

# Registering executables
mkdir -p $HOME/bin

ln -s $PWD/venv/bin/ldap_admin $HOME/bin/ldap_admin
ln -s $PWD/venv/bin/ldap_user $HOME/bin/ldap_user

deactivate
cd $HOME
```

## Quick usage

Associated commands,

* `ldap_admin`: Permite listar, crear y eliminar usuarios del directorio LDAP.
* `ldap_user`: Herramientas para el usuario final, permite validar su clave y cambiarla.
