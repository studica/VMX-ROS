#!/bin/bash -e

##########
##
## ROS Noetic install script
##
## V2 (09/08/2021) for RPI 4
##
## By: James Taylor
##
## Expected Time to run: 3 hrs
##########

#Clear SECONDS for an elapsed Timer
SECONDS=0

#Start Script
echo -e "\e[31m Welcome to the ROS noetic install script \e[0m"
echo -e "\e[31m Version 2 \e[0m"
echo -e "\e[31m Created by James \e[0m"
sleep 1

echo -e "\e[31m adding swap space \e[0m"
sleep 5
sudo dphys-swapfile swapoff
sudo sed -i 's/CONF_SWAPSIZE=[0-9]\+$/CONF_SWAPSIZE=1024/g' /etc/dphys-swapfile
sudo dphys-swapfile setup
sudo dphys-swapfile swapon

current=$SECONDS
echo -e "\e[31m Time Elapsed: $(($current / 60)) minutes and $(($current % 60)) seconds \e[0m"

echo -e "\e[31m Grabbing Noetic Source List \e[0m"
sleep 5
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

current=$SECONDS
echo -e "\e[31m Time Elapsed: $(($current / 60)) minutes and $(($current % 60)) seconds \e[0m"


echo -e "\e[31m Setup Noetic Key \e[0m"
sleep 5
#sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

current=$SECONDS
echo -e "\e[31m Time Elapsed: $(($current / 60)) minutes and $(($current % 60)) seconds \e[0m"


echo -e "\e[31m Pull meta data \e[0m"
sleep 5
sudo apt-get update --allow-releaseinfo-change

current=$SECONDS
echo -e "\e[31m Time Elapsed: $(($current / 60)) minutes and $(($current % 60)) seconds \e[0m"

echo -e "\e[31m Build dependencies \e[0m"
sleep 5
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake

current=$SECONDS
echo -e "\e[31m Time Elapsed: $(($current / 60)) minutes and $(($current % 60)) seconds \e[0m"

echo -e "\e[31m ROS Setup \e[0m"
sleep 5
sudo rosdep init
rosdep update

current=$SECONDS
echo -e "\e[31m Time Elapsed: $(($current / 60)) minutes and $(($current % 60)) seconds \e[0m"

echo -e "\e[31m Fetch ROS \e[0m"
sleep 5
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
rosinstall_generator desktop --rosdistro noetic --deps --wet-only --tar > noetic-desktop-wet.rosinstall
wstool init src noetic-desktop-wet.rosinstall
rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster

current=$SECONDS
echo -e "\e[31m Time Elapsed: $(($current / 60)) minutes and $(($current % 60)) seconds \e[0m"

echo -e "\e[31m installing some packages for required before starting \e[0m"
sleep 5
sudo apt-get update
sudo apt-get install -y python3-catkin-pkg python3-empy python3-yaml python3-defusedxml python3-rospkg python3-netifaces

current=$SECONDS
echo -e "\e[31m Time Elapsed: $(($current / 60)) minutes and $(($current % 60)) seconds \e[0m"

echo -e "\e[31m Compile ROS \e[0m"
sleep 5
sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 

current=$SECONDS
echo -e "\e[31m Time Elapsed: $(($current / 60)) minutes and $(($current % 60)) seconds \e[0m"

echo -e "\e[31m removing built up files \e[0m"
sleep 5
sudo apt-get clean
sudo apt-get autoremove -y

current=$SECONDS
echo -e "\e[31m Total time taken to run this script: $(($current / 60)) minutes and $(($current % 60)) seconds \e[0m"

echo -e "\e[31m All Done Reboot Required"
sleep 5
while read -r -t 0; do read -r; done
read -p "Reboot now? (Y/n) " -n 1 -r 
echo # moves to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
   sudo reboot
fi 
echo -e "\e[0m]"
