# Nevada Nano Gas Detector ROS Driver

![detector_image](https://github.com/unr-arl/nv_nano_gas_detector_ros/blob/master/nv_nano_detector.jpg)


This is a ROS node wrapping the Nevada Nano UART API for obtaining gas concentration and ID measurements from the Nevada Nano Methane Gas Detector. This driver queries the sensor at 0.5Hz and publishes the concentration of gas detected, the ID of the gas detected, as well as a custom message containing all of the information output from the sensor (concentration, id, temperature, humidity, etc). The driver is heavily based on the UART Flam Example provided by Nevada Nano, and wraps their communication functions. The driver has no dependencies other than ROScpp and standard C libraries.

#### Topics:
  - /gas_detect/concentration - (std_msgs::Float32), denotes the gas concentration as the % Lower Explosive Limit.
  - /gas_detect/gas_id - (std_msgs::UInt32), contains the gas identified
  - /gas_detect/msg - (custom - gas_detect_driver_node::gas_detect)

#### Launch Parameters
  - Serial port name (eg. '/dev/ttyUSB')
  - Serial port number (eg. 0)
