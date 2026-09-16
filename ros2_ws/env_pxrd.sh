# Source this in EVERY terminal you use for the PXRD cell:
#   source ~/Ks_FR/ros2_ws/env_pxrd.sh
#
# Puts this terminal on ROS_DOMAIN_ID 43, isolated from the unchained cell
# (which runs on 42). Run unchained and pxrd sims side by side with zero
# DDS interference.
export ROS_DOMAIN_ID=43
echo "[pxrd] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
