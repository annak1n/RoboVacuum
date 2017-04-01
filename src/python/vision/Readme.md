# Vision module

This is an attempts at using a single camera facing upwards from the robot to perform stereovision and from this determine some "global position". The additon of a fisheye lens might also help some object detection.

## TODO:
- Get SCI-KIT image working on RPI, bersion I have seems out of date (?)
..* required the removal of sudo apt get cython and installation of pip python

- The alogorith to perform stero vision and get point cloud
- Registering point cloud onto "globla view"


## Algorithm:
    Basic idea:

- With unknown position take a photo, move precise distance in a straight line (limits not known yet) and take another photo. Both images should have Canny edge detection perfomed to reduce the load for the stero calcuation.

- Pefrom stero image registration by performing a normalised cross correlation (NCC)  along a line search from primary image to secondary image. Based upon this and the camera matrix a point cloud is generated.

- A RANSAC(?) is used to combined the new point cloud with an existing map. The tranformation of this combination can then be used to determine a global position

## smarter algorithm
The ceiling will always be aproximatly the same height. The pseudo sterovision is likely not needed as it is then possible to create an angle map of pixels and look for features inside the "ceiling" planes intersection with the roof.