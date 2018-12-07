#!/bin/bash

# Make sure we don't do anything stupid!
cd ~

# Delete useless folders
rmdir Documents
rmdir Music
rmdir Pictures
rmdir Public
rmdir Templates
rmdir Videos
rm examples.desktop

# Install stuff that people in the lab like to use
sudo apt-get install -yq \
  curl \
  emacs \
  gitk \
  terminator \
  vim \
  wget

# Allow SSH into this machine
sudo apt-get install -yq openssh-server


# Set the default settings for gedit to be OK for programming
gsettings set org.gnome.gedit.preferences.editor auto-indent true
gsettings set org.gnome.gedit.preferences.editor bracket-matching true
gsettings set org.gnome.gedit.preferences.editor display-line-numbers true
gsettings set org.gnome.gedit.preferences.editor highlight-current-line true
gsettings set org.gnome.gedit.preferences.editor insert-spaces true
gsettings set org.gnome.gedit.preferences.editor tabs-size "uint32 2"

# Clean up the Unity launcher
gsettings set com.canonical.Unity.Launcher favorites "['application://firefox.desktop']"

