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
