#!/usr/bin/bash

# setup_pi.sh
#
# Helper script to setup a brand-new Raspberry Pi 5 running Ubuntu 24.04.

# 1 - Uninstall unneeded programs
echo "Uninstalling unneeded programs..."

# Thunderbird email client
echo "Removing Thunderbird..."
sudo snap remove --purge thunderbird
echo "Done removing Thunderbird"

# Transmission torrent client
echo "Removing Transmission..."
sudo apt-get remove transmission-gtk
echo "Done removing Transmission"

# Libreoffice
echo "Removing Libreoffice..."
sudo apt-get remove --purge "libreoffice*"
echo "Done removing Libreoffice"

echo "Done removing unneeded programs"

# 2 - Install ROS 2 Jazzy.  Following the steps from:
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
echo "Installing ROS 2 Jazzy"

# Set up locales
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add Ubuntu Universe repo
sudo apt install software-properties-common
sudo add-apt-repository universe

# Set up ROS 2 Apt package
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ROS 2 development tools
sudo apt update
sudo apt upgrade
sudo apt install ros-dev-tools ros-jazzy-desktop

echo "Done installing ROS 2 Jazzy"

# 3 - Install additional packages
echo "Installing additional packages..."

sudo apt install -y \
  cppcheck \
  git \
  i2c-tools \
  valgrind \
  vim

echo "Done installing additional packages"
echo "Raspberry Pi 5 setup is complete!"
