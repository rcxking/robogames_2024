#!/usr/bin/bash
echo "Now installing dependencies"

# Update repos
sudo apt update

# Add required dependencies here
declare -a dependencies=(
  "ros-noetic-robot-localization"
)

# Install dependencies
for depend in "${dependencies[@]}"
do
  echo "Installing: $depend"
  sudo apt install -y $depend
done

echo "Done installing dependencies"
