# sht4x driver
A ROS2 wrapper for the Adafruit SHT4x python sensor driver and library

## Install
(from your venv)
```
pip install -r requirements.txt
colcon build
source install/setup.zsh
```

## Run
```
ros2 run sht4x sht4x_node

ros2 topic echo /sht4x/temperature
ros2 topic echo /sht4x/relative_humidity
```

## Dependencies
This depends on the following python packages:
- [`adafruit-circuitpython-sht4x`](https://github.com/adafruit/Adafruit_CircuitPython_SHT4x) 
python library for SHT4x temperature / humidity sensors 
- [`empy`](https://pypi.org/project/empy/) version 3.3.4 
(required to workaround [this issue](https://github.com/colcon/colcon-core/issues/602) 
until `colcon` supports empy v4)