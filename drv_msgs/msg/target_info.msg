Header header

## Readable labels of object (defined by PASCAL-VOC)
# 'aeroplane', 'bicycle', 'bird', 'boat',
# 'bottle', 'bus', 'car', 'cat', 'chair',
# 'cow', 'diningtable', 'dog', 'horse',
# 'motorbike', 'person', 'pottedplant',
# 'sheep', 'sofa', 'train', 'tvmonitor'
std_msgs/String label

## Name for person
std_msgs/String name

## what to do to the target, 
## includes 'find' 'follow' 'approach' 'grasp' 'put'
std_msgs/String action

## hsv
std_msgs/UInt16MultiArray hsv_color

## The target's current location
std_msgs/String location_curr

## The target's target location
std_msgs/String location_tgt

## Object's geometry
geometry_msgs/Point size
## Distance between object's centroid to its base (for putting)
std_msgs/Float32 center_to_base
