![alt text][logo] PCL OpenNI Experimentation
============

A simple playground for experimenting with the PCL library and OpenNI.

Key Dependencies
------------
* [PCL](https://github.com/PointCloudLibrary/pcl) - An *uber* standalone library for handing 2D/3D image and point cloud processing.
* [OpenNI](https://github.com/OpenNI/OpenNI) - An interfacing library for streaming point cloud data from physical OpenNI compliant sensors.
* [Eigen](http://eigen.tuxfamily.org/index.php) - A C++ linear algebra template library.

Command Line Arguments
------------
| Flag      | M/O | Description  |
| ----------|:---:|------------: |
| `-h`        | O   | Help: Lists out the available command line args. |
| `-s`        | O   | Stream data from the first OpenNI compliant sensor found over USB. |
| `-f [file]` | O   | The path to a [PCD file](http://pointclouds.org/documentation/tutorials/pcd_file_format.php#pcd-file-format) to visualize. |
| `-filter`   | O   | Attempts to filter noise & invalid points from the inputted point cloud file. |
| `-hullCalc` | O   | *(Incomplete)* Builds a convex hull from the inputted PCD file and calculated surface area & volume. |

**Notes:**
* Pressing `space` during a stream will automatically package up the point cloud and output the PCD file into the execution directory.
* The PCL viewer allows you to pan and rotate the point cloud.
* A range image is built and visualized for PCD files inputted into the tool.

Installation
------------
1. Clone the repo and install PCL with it's associated dependencies.
2. Ensure [CMake](https://cmake.org) and [GNU Make](https://www.gnu.org/software/make/) are installed.
2. Execute the following from within the root directory of the repository:
```sh
mkdir build     # Create a new build directory
cd build
cmake ..        # Resolve dependencies & generate the Makefile for the project
make
./openni_viewer
```

References
------------
* [PCL Docs](http://docs.pointclouds.org/trunk/index.html)
* [OpenNI Grabber Framework Primer](http://pointclouds.org/documentation/tutorials/openni_grabber.php)
* [Constructing Hull Polygons](http://www.pointclouds.org/documentation/tutorials/hull_2d.php)
* [Point Cloud Background Subtraction](http://answers.ros.org/question/36272/background-subtraction-of-pointcloud/)
* [Constructing Range Images from Point Clouds](http://pointclouds.org/documentation/tutorials/range_image_creation.php)
* [Visualizing Range Images](http://pointclouds.org/documentation/tutorials/range_image_visualization.php)
* [OctoMap - 3D Mapping Framework Based on Octrees](http://octomap.github.io)
* [All the Awesome PCL Tutorials](http://pointclouds.org/documentation/tutorials/)

[logo]: icon64.png "PCL Logo"
