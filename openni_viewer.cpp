/* \author Geoffrey Biggs */


#include <iostream>
#include <mutex>

#include <pcl/io/openni_grabber.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>


using namespace pcl;

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
boost::shared_ptr<pcl::visualization::PCLVisualizer> main_viewer;
std::mutex m;

float angular_resolution_x = 0.5f,
      angular_resolution_y = angular_resolution_x;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool live_update = false;

Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
float noise_level = 0.0;
float min_range = 0.0f;
int border_size = 1;

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sCloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sCloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sCloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sCloud");
//   viewer->addCoordinateSystem (1.f);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> initializedViewer ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (1, 1, 1);
  viewer->initCameraParameters ();
  return viewer;
}

void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}

// Grabber point cloud callback
void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
	m.lock();
 	if (!main_viewer->wasStopped()) {

		// The resulting cloud from the PCL Kinect Grabber is upside down
		// Transform the cloud into the correct orientation
		// Reference: http://pointclouds.org/documentation/tutorials/matrix_transform.php
		float theta = M_PI; // The angle of rotation in radians
		
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		
		// Theta radians arround Z axis
		transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
		
		// Executing the transformation
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ());
		pcl::transformPointCloud (*cloud, *transformed_cloud, transform);

		point_cloud_ptr = transformed_cloud;

		// Update the viewer's point cloud
		// NOTE: This cannot be done at the same time as the "spin" renderer
		//		 Protect against a collision via a mutex
		// Reference: http://stackoverflow.com/questions/9003239/stream-of-cloud-point-visualization-using-pcl
		main_viewer->updatePointCloud(transformed_cloud, "sCloud");
 	}
	m.unlock();
}

void printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-k           Source auto steaming data from Kinect\n"
            << "-f [file]    Visualize the specified source file\n"
            << "\n\n";
}

int outputSequence = 9;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	
	if (event.keyDown())
	{
		std::cout << "Pressed: '" << event.getKeySym() << "' \n";
	
		if (event.getKeySym() == "space")
		{
			//m.lock();
			std::string outputFileName = "outputCloud" + std::to_string(outputSequence) + ".pcd";
			pcl::io::savePCDFileASCII(outputFileName, *point_cloud_ptr);
			outputSequence ++;
			std::cout << "Writing '" << outputFileName << "' ... \n";
			//m.unlock();
		}
	}
}

int main (int argc, char** argv)
{
	if (pcl::console::find_argument (argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return 0;
	}
	
	
	bool sensorStream = false;
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>& point_cloud = *point_cloud_ptr;
	
 	main_viewer = rgbVis(point_cloud_ptr);
	main_viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&main_viewer);
	
	angular_resolution_x = pcl::deg2rad (angular_resolution_x);
  	angular_resolution_y = pcl::deg2rad (angular_resolution_y);
  	
  	
  	if (pcl::console::find_argument (argc, argv, "-k") >= 0)
	{
		std::cout << "Mode: Stream Data from Kinect Sensor\n";
		sensorStream = true;
	}
	else if (pcl::console::find_argument (argc, argv, "-f") >= 0)
	{
		std::string inputfile = argv[argc - 1];
		std::cout << "Mode: Visualize Static PCD File " << inputfile << "\n";
		
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (inputfile, point_cloud) == -1)
		{
			PCL_ERROR("Couldn't read PCD file \n");
			return (-1);
		}
		
		main_viewer->updatePointCloud(point_cloud_ptr, "sCloud");
		std::cout << "Successful read of " << inputfile << ".\n";
	}
  	
  	// TEMP: Build Static Cloud for Range Image
  	for (float x=-0.5f; x<=0.5f; x+=0.01f)
    {
      for (float y=-0.5f; y<=0.5f; y+=0.01f)
      {
        pcl::PointXYZRGBA point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
        point_cloud.points.push_back (point);
      }
    }
    point_cloud.width = (int) point_cloud.points.size ();  point_cloud.height = 1;
	
	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;   
	range_image.createFromPointCloud (point_cloud, angular_resolution_x, angular_resolution_y,
									pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
									scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	
	// ....
	// main_viewer = initializedViewer();
// 	pcl::visualization::PCLVisualizer viewer = *main_viewer;
//   	
// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
// 	viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
// 	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
	//viewer.addCoordinateSystem (1.0f, "global");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
	//viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
	
// 	setViewerPose(viewer, range_image.getTransformationToWorldSystem ());
	// ....
	
	pcl::visualization::RangeImageVisualizer range_image_widget("Range Image");
	range_image_widget.showRangeImage(range_image);
	
	
	if (sensorStream) {
		pcl::Grabber* interface;
		try {
			interface = new pcl::OpenNIGrabber();
		} catch (const pcl::IOException& exp) {
			PCL_ERROR("ERROR: Failed to find or initialize an input OpenNI device. Please verify that the device is connected. \n");
			std::cout << "Exception Details: " << exp.what() << endl;
			return 0;
		}

		std::cout << "Interface " << interface << "\n";

		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&::cloud_cb_, _1);

		interface->registerCallback(f);

		interface->start();

		std::cout << "Starting... \n";

		while (!main_viewer->wasStopped())
		{
			m.lock();
			main_viewer->spinOnce(100);
			range_image_widget.spinOnce();

			if (live_update)
			{
				scene_sensor_pose = main_viewer->getViewerPose();
				range_image.createFromPointCloud(point_cloud, angular_resolution_x, angular_resolution_y,
												pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
												scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);
				range_image_widget.showRangeImage(range_image);
			}
		
			m.unlock();
			boost::this_thread::sleep (boost::posix_time::microseconds (100));
		}

		interface->stop();
	} else {
		while (!main_viewer->wasStopped())
		{
			main_viewer->spinOnce(100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100));
		}
	}
}