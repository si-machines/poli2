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

# Dependencies for Sublime Text (from https://www.sublimetext.com/docs/3/linux_repositories.html#apt)
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
sudo apt-get install -yq apt-transport-https
echo "deb https://download.sublimetext.com/ apt/dev/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
sudo apt-get update

# Install stuff that people in the lab like to use
sudo apt-get install -yq \
  curl \
  emacs \
  gitk \
  sublime-text
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

# Set the cache to timeout after 1 hour (setting is in seconds)
git config --global credential.helper 'cache --timeout=3600'
