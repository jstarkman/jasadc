# C.H.I.P as an ADC

1. [Flash CHIP](https://flash.getchip.com/) with 4.4 headless (requires Chromium-family browser).
2. Upgrade to Debian Stretch if desired.
3. Install ROS1 or ROS2 via [pre-compiled tarballs](https://github.com/jstarkman/jasadc/releases).
4. Run:
	```
	ROS1: $ ROS_MASTER_URI=<your IP address here> rosrun jasadc main
	ROS2: $ ros2 run jasadc main
	```
5. See results being published.
