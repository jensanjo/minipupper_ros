#!/bin/sh

echo "Configure camera"

# rotate the image by 180°
v4l2-ctl -d /dev/video0 --set-ctrl rotate=180