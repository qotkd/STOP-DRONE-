This software has two objects, exploit.py and test_test.py.
exploit.py attack a drone or WIFI AP. exploit.py can sniff WIFI AP, deauth attack, connect WIFI, Sniff WIFI, and do ARP Spoofing.
test_test.py use pymavlink, you can send a message to drone which is connected.
You can armed/disarmed a drone, control a drone to arrow buttons.

This software is created by MAC OS X, so you want to execute this software, please use MAC OS X.
If you use UNIX or Windows, you need to reprograming about airport.
Unix use 'iwconfig' to use wlan monitor mode, windows need to use 'netsh'.
Unix: iwconfig wlan0 mode monitor
windows: netsh interface set interface "Wireless Network Connection"

Use python 3.7 ~ 3.10.
Older python version has bug which is not detect 'libpcap' on MAC OS X, so you need to download latest versions or after 2022.
Python 3.11 has a bug which can't install mavproxy. If this bug is fixed, you can use this software.

You need to import Scapy, mavproxy, keyboard, libpcap.

pip install scapy
pip install mavproxy
python -m pip install keybaord
brew install libpcap

