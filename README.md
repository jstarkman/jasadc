# NTC C.H.I.P. as an ADC

1. [Flash CHIP](https://flash.getchip.com/) with 4.4 headless (requires Chromium-family browser).
2. [dist-upgrade](https://bbs.nextthing.co/t/is-ntc-ready-for-the-move-from-debian-jessie-to-stretch/14706/9) to Debian Stretch if desired.
3. Install ROS1 or ROS2 from scratch or via [pre-compiled tarballs](https://github.com/jstarkman/jasadc/releases).
4. Run:
	```
	ROS1, local roscore:  $ rosrun jasadc main
	ROS1, remote roscore: $ ROS_MASTER_URI=http://hostname:11311/ rosrun jasadc main
	ROS2, LAN multicast:  $ ros2 run jasadc main
	```
5. See results being published.

Note that the ROS2 tarball was built under Stretch (Debian 9) and may not work under the default Jessie (Debian 8).
