#!/bin/bash
# Set the V4L2 controls of the Logitech C310 before usb_cam starts.
#
# usb_cam's own auto_white_balance / autoexposure / autofocus parameters map to
# control names this camera does not have ("unknown control ..." in its log),
# so they are ignored. These names come straight from `v4l2-ctl --list-ctrls`
# on the C310.
#
# Why set them at all: FAST-LIVO2 fuses image patches photometrically, and it
# estimates exposure online (vio.exposure_estimate_en), but a webcam's auto
# exposure ramps gain too, and every ramp is a brightness change the estimator
# has to absorb. A fixed exposure indoors behaves much better.
#
# Usage:
#   set_camera_controls.sh [device] [manual|auto] [exposure_units_of_100us] [brightness]
# Defaults: /dev/video0 manual 285 120   (= 28.5 ms exposure, brightness 120)
#
# Exposure: at 30 fps the frame budget is 33.3 ms = 333 units, so 285 is what
# an ordinary indoor room needs; it is also as high as it can go without
# opening up motion blur on a handheld rig (0.5 m/s x 28.5 ms = 14 mm).
# Raise it only if the room is darker than that; drop it (or frame rate) if
# the map looks smeared rather than grainy.
#
# Brightness: this is the control that actually decides whether the image is
# usable, and it is NOT the same as exposure -- it is a digital offset, so it
# lifts the blacks without lengthening the exposure. It also has nothing to do
# with what this script used to leave alone: the driver's default is 128, and a
# C310 that had 50 written into it (usb_cam's default) looks hopelessly dark no
# matter what the exposure says. Measured here, exposure 285 fixed:
#   brightness  50 -> luma mean 42,  median  44   (too dark)
#   brightness 120 -> luma mean 96,  median 105   (good, 0% clipped)
#
# The value read back can differ from the one written: this C310 quantises
# exposure (asking for 100 gives 105). That is the camera, not an error.
#
# Settings do not survive the camera being unplugged/replugged, so run this
# after every plug-in, before the capture node starts.
set -u

DEV=${1:-/dev/video0}
MODE=${2:-manual}
EXPO=${3:-285}
BRIGHT=${4:-120}

if ! v4l2-ctl -d "$DEV" --list-ctrls >/dev/null 2>&1; then
  echo "no V4L2 device at $DEV (is the camera plugged in?)" >&2
  exit 1
fi

set_ctrl() {
  if ! v4l2-ctl -d "$DEV" --set-ctrl="$1" 2>/tmp/set_camera_controls.err; then
    echo "  warning: could not set $1: $(tr -d '\n' < /tmp/set_camera_controls.err)" >&2
  fi
}

# 1 = 50 Hz, 2 = 60 Hz. 50 Hz for mains-powered lighting in CN/EU; a mismatch
# shows up as horizontal banding in every frame.
set_ctrl power_line_frequency=1

if [ "$MODE" = "manual" ]; then
  set_ctrl white_balance_automatic=0
  set_ctrl white_balance_temperature=4600
  set_ctrl auto_exposure=1
  set_ctrl exposure_time_absolute="$EXPO"
else
  set_ctrl white_balance_automatic=1
  set_ctrl auto_exposure=3
fi

# Exposure and brightness are independent: brightness is a digital offset, so
# it belongs in both modes. Left unset, it stays whatever the driver last had
# -- which is how a stuck 50 made the image look dark with the exposure already
# maxed out.
set_ctrl brightness="$BRIGHT"

# Do not let a long exposure pull the frame rate down: LIVO2's association is
# much happier with a steady 30 Hz.
set_ctrl exposure_dynamic_framerate=0

echo "=== $DEV ==="
v4l2-ctl -d "$DEV" --get-ctrl=power_line_frequency,white_balance_automatic,\
white_balance_temperature,auto_exposure,exposure_time_absolute,\
exposure_dynamic_framerate,gain,brightness
