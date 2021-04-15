#include <iostream>
#include <cstring>
#include "PointCloud.h"


pcl::PointCloud<pcl::PointXYZRGB>::Ptr read_pcd(const char* filename)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZRGB>(filename, *cloud);
	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr read_ply(const char* filename)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(filename, *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file %s\n", filename);
		cloud = nullptr;
		exit(EXIT_FAILURE);
	}
	return cloud;
}

void print_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int num_of_points)
{
	if (num_of_points > cloud->points.size())
		num_of_points = cloud->points.size();
	for (size_t i = 0; i < num_of_points; ++i)
	{
		cout << "point #" << i + 1
			<< "\tx: " << cloud->points[i].x
			<< "\ty: " << cloud->points[i].y
			<< "\tz: " << cloud->points[i].z
			<< "\t\tr: " << (int)cloud->points[i].r
			<< "\tg: " << (int)cloud->points[i].g
			<< "\tb: " << (int)cloud->points[i].b
			<< endl;
	}
}

void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = std::make_shared<pcl::visualization::PCLVisualizer>("Showing Point Cloud");
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, target_color, "Target Point Cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Target Point Cloud");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}
}

void visualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool rgb)
{
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = std::make_shared<pcl::visualization::PCLVisualizer>("Showing Point Cloud");
	if (!rgb)
	{
		// set background color to black
		viewer->setBackgroundColor(0, 0, 0);
		// set point color to red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>target_color(cloud, 255, 0, 0);
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud, target_color, "Target Point Cloud");
	}
	else
	{
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "Target Point Cloud");
	}
	
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Target Point Cloud");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}
}

void save(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const char* filename)
{
	pcl::PCDWriter writer;
	writer.write(filename, *cloud);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pathrough_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
	const char* field, float min_value, float max_value)
{
	//pcl::PointXYZRGB min_pt, max_pt;
	//pcl::getMinMax3D(*cloud, min_pt, max_pt);
	
	if (!(strcmp(field, "x") == 0 || strcmp(field, "y") == 0 || strcmp(field, "z") == 0))
	{
		PCL_ERROR("Field error, please select from [\"x\", \"y\", \"z\"]");
		exit(EXIT_FAILURE);
	}

	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setFilterFieldName(field);
	pass.setInputCloud(cloud);
	pass.setFilterLimits(min_value, max_value);
	pass.setFilterLimitsNegative(false);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pass.filter(*cloud_filtered);
	
	return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float lx, float ly, float lz)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(lx, ly, lz);
	sor.filter(*cloud_filtered);
	return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr statistical_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int n, double stddev)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(n);
	sor.setStddevMulThresh(stddev);
	sor.filter(*cloud_filtered);
	return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr radius_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double radius, int min_pts)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(radius);
	outrem.setMinNeighborsInRadius(min_pts);
	outrem.setKeepOrganized(false);
	outrem.filter(*cloud_filtered);
	return cloud_filtered;
}
