# ROV Multiserver

One-line installer:

```bash
cd $HOME

git clone http://gitlab+deploy-token-42:g17YggPyWeLL8rTmAE6k@gitlab.telemcloud.cl/aquarovdevelopment/rov-multiserver.git && \
cd rov-multiserver && python3 -m venv venv && . ./venv/bin/activate && \
pip install -U pip setuptools wheel && \
pip install -e .

# Registering executables
mkdir -p $HOME/bin

ln -s $PWD/venv/bin/rov-server $HOME/bin/rov-navigation

deactivate
cd $HOME

```