#!/usr/bin/env bash

git submodule update --init --recursive

SCRIPT_PATH=`pwd`

cd "${SCRIPT_PATH}/lib/BreezySLAM/python"
sudo python setup.py install

cd "${SCRIPT_PATH}/lib/BreezyLidar/python"
sudo python setup.py install

pip install -r "${SCRIPT_PATH}/requirements.txt"