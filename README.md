# *EXTARIA* : Extensible Teleoperation Architecture Optimized for




### **Dependencies**

| **Tool**             | **Version**                              |
| :------------------- | :--------------------------------------- |
| **Operating System** | Debian 11.3 (64 bit) realtime            |
| **Touch Driver**     | v2022_04_04                              |
| **OpenHaptics**      | v3.4.0                                   |
| **QT**               | 5.11.3                                   |


### **Touch Setup**
1. Get Touch Driver and OpenHaptics [here](https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US) and extract folders.

2. Install TouchDriver and OpenHaptics dependencies:
~~~
sudo apt install qt5-default build-essential libncurses5 libncurses5-dev freeglut3 freeglut3-dev zlib1g-dev
~~~

3. Install openhaptics
~~~
cd ~/Downloads/openhaptics_3.4-0-developer-edition-amd64
sudo ./install
~~~

4. Copy drivers
~~~
cd ~/Downloads/TouchDriver2022_04_04/
sudo cp usr/lib/libPhantomIOLib42.so /usr/lib
sudo cp bin/Touch* /usr/bin/
~~~

5. Connect the device and use ListUSBHapticDevices script to find out which port the haptic device has been assigned and then set permission for the com port. NOTE: this procedure must be done every time user disconnects and re-connects the device.
~~~
cd ~/Downloads/TouchDriver2022_04_04/
bash ListUSBHapticDevices
sudo chmod 777 /dev/ttyACM0 
~~~

6. (Optional) To access to COM port devices without having to CHMOD each time run:
~~~
sudo adduser YourUserName dialout
~~~

7. run Touch Setup and Touch Diagnostic.
~~~
echo 'export GTDD_HOME=~/.3dsystems' >> ~/.bashrc
source ~/.bashrc
Touch_Setup
Touch_Diagnostic
~~~

5. (Optional) Copy an example file
~~~
cd /opt/OpenHaptics/Developer/3.4-0/examples/HD/console/QueryDevice/
make
sudo cp QueryDevice /usr/bin/Touch_QueryDevice
~~~



### **Authors:**
- Enrico Sgarbanti
- Diego Dall'Alba
- Fabrizio Boriero

Thanks to [Nimbo](http://www.nimbo-srl.com) and [Altair](https://metropolis.scienze.univr.it) for the collaboration.

---
### **License**
This project is licensed under the GPL v3 License - see the [LICENSE.md](LICENSE.md) file for details.
