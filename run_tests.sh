#!/usr/bin/env bash
set -e

if [ ! -d ".venv" ]; then
    python3 -m venv --system-site-packages .venv
fi

source .venv/bin/activate

pip install --upgrade pip
pip install pytest coverage
pip install -e avai_lab

source /opt/ros/humble/setup.bash
cd race_car_ws && colcon build && cd ..
source race_car_ws/install/setup.bash

python -m coverage run -m pytest unit_tests/
python -m coverage report
python -m coverage html

deactivate