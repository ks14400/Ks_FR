# Source this in EVERY terminal you use for the UNCHAINED cell:
#   source ~/Ks_FR/ros2_ws/env_unchained.sh
#
# Puts this terminal on ROS_DOMAIN_ID 42. Nodes on a different domain (e.g.
# the PXRD cell on 43) are completely invisible — no interference.
export ROS_DOMAIN_ID=42
echo "[unchained] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
