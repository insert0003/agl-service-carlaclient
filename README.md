# agl-service-carlaclient

![](https://oss-project.tmc-tokai.jp/gitlab/AppsDev/hmi-fw-v1.1/carla-server/raw/master/icon.png)  

AGL client end for communication with carla server.
------
[![](https://img.shields.io/badge/Nexty-Dalian-red.svg?style=popout&logo=appveyor)](http://www.dl.cn.nexty-ele.com/cn/)
### 🗃️ How to run:
* **Step 1: Build with AGL SDK**
    ```bash
    mkdir build
    cmake ..
    make
    ```
    
* **Step 2: Copy config files**  
    ```bash
    cp ./conf/*.json /etc
    cp ./conf/dev-mapping.conf /etc
    cp ./conf/vcan.sh /usr/bin
    chmod u+x /usr/bin/vcan.sh
    ```
    copy `./conf/can-dev.service` to systemd folder and enable it  
    ```bash
    scp conf/can-dev.service root@XXX:/lib/systemd/system
    systemctl enable can-dev.service
    ```
    replace the ip with actual PC ip in `carla-server.json`  
    replace with actual can port in `dev-mapping.conf`  
    copy `dummy_gps.txt` to `/etc`  
    
* **Step 3: Install wgt package**
    ```bash
    afm-utils install carla-client-service.wgt
    ```
