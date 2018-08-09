#!/bin/bash

echo "downloading.."
wget --no-check-certificate "https://drive.google.com/uc?id=0B8X8lwfossYQLUhEYmJGQzNFSFU&export=download" -O test.zip
# unzip the downloaded file
unzip test.zip -d /home/sar/catkin_ws/src/sar_core/testing
rm test.zip
rm -rf testing/__MACOSX
