#!/bin/sh

#####################################################################################
# basic_setup.sh
#
#   Script to bootstrap this repo for running the JS games only (no robot/CoRDial setup). 
#
#####################################################################################
sudo apt-get update

echo "Installing NodeJS and NPM"
# Install Node and NPM
sudo apt-get install -y nodejs npm

# Setup Node.js
sudo ln -s /usr/bin/nodejs /usr/bin/node

# Install global Node packages (gulp and http-server)
sudo npm install --global gulp-cli http-server

echo "Basic setup complete."
