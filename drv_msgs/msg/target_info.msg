Header header

# readable labels (with default rcnn)
# 'aeroplane', 'bicycle', 'bird', 'boat',
# 'bottle', 'bus', 'car', 'cat', 'chair',
# 'cow', 'diningtable', 'dog', 'horse',
# 'motorbike', 'person', 'pottedplant',
# 'sheep', 'sofa', 'train', 'tvmonitor'
std_msgs/String label

# name for person
std_msgs/String name

# what to do to the target, includes find follow approach grasp drop
std_msgs/String action

# hsv
std_msgs/UInt16MultiArray hsv_color
std_msgs/String location

