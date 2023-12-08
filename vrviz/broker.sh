#!/bin/bash

conf=$(ros2 pkg prefix vrviz)/share/vrviz/config/mosquitto.conf
port=$VRVIZ_MQTT_BROKER_PORT

tempconf=$HOME/.ros/mqtt/temp
mkdir -p $tempconf
cp $conf $tempconf/mosquitto.conf
echo "listener $port" >> $tempconf/mosquitto.conf

mosquitto -c "$tempconf/mosquitto.conf"
