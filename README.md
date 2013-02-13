# RoboCup@Work Safe Object Transportation

This node is responsible for ensuring that objects are safely moved from one location to another without falling off the platform or moving in a manner that makes it difficult or impossible to grasp the objects once the system has arrived at its destination. 

## Input

This node takes in an RGBD point cloud as provided from a sensor system such as the Microsfot Kinect. 
## Output

This node will provide a message that will notify the platform if the objects are moving to much or if they have become disturbed. 

## Future Developments

* Notify the system as objects move so that velcoties can be adjusted to account for the disturbances. 