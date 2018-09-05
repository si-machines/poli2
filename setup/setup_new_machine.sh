#!/bin/bash

# Install stuff that people in the lab like to use
sudo apt-get update
sudo apt-get install -yq \
  curl \
  emacs \
  libusb-1.0-0 \
  terminator \
  vim \
  wget \

# Allow SSH into this machine
sudo apt-get install openssh-server

# Set the default settings for gedit to be OK for programming
gsettings set org.gnome.gedit.preferences.editor display-line-numbers true
gsettings set org.gnome.gedit.preferences.editor highlight-current-line true
gsettings set org.gnome.gedit.preferences.editor bracket-matching true
gsettings set org.gnome.gedit.preferences.editor tabs-size "uint32 2"
gsettings set org.gnome.gedit.preferences.editor auto-indent true
gsettings set org.gnome.gedit.preferences.editor insert-spaces true

