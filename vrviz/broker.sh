#!/bin/bash

# Identify config information
conf=$(ros2 pkg prefix vrviz)/share/vrviz/config/mosquitto.conf
port=$VRVIZ_MQTT_BROKER_PORT

# Copy conf to temp directory and add port details
tempconf=$HOME/.ros/mqtt/temp
mkdir -p $tempconf
cp $conf $tempconf/mosquitto.conf
echo "listener $port" >> $tempconf/mosquitto.conf

mosquitto -c "$tempconf/mosquitto.conf"
