#!/bin/bash


cd
cd  1TBSSD/Documentos/JETSON_AQUIRE_LIDAR_GNSS_IMU/



echo "CHECKK INTERNET CONNECTION... "
wget -q --spider http://google.com

if [ $? -eq 0 ]; then
    echo "ONLINE"
else
    echo "OFFLINE"
fi


git pull

echo  "UPDATED SUCCESFULLY"
echo "STARTING jetson SYSTEM "


python3 main.py