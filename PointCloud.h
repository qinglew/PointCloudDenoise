/*
 * PointCloud.h
 * @author: Carlos Leo
 * @date: 2021-04-14
 */
#pragma once
#ifndef __POINTCLOUD_H__
#define __POINTCLOUD_H__

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

/*
 * Read .pcd file as point cloud.
 * @param filename path of .pcd path
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr read_pcd(const char* filename);

/*
 * Read .ply file as point cloud.
 * @param filename: path of .pcd path
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr read_ply(const char* filename);

/*
 * Output the coordinate (x, y, z) and (r, g, b) of points in point cloud.
 * @param cloud: point cloud object
 * @param num_of_points: the number of points you want to output
 */
void print_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int num_of_points);

/*
 * Visualize a point cloud without color information.
 */
void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

/*
 * Visualize a point cloud with color information.
 * @param cloud: point cloud
 * @param rgb: do not visualize the color in default
 */
void visualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool rgb = false);

/*
 * Save point cloud object into file.
 * @param cloud: point cloud object
 * @param filename: the target path
 */
void save(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const char* filename);

/*
 * Pathrough filter.
 * @param cloud: point cloud object
 * @param field: "x", "y" or "z", which means the axis to be path through
 * @param min_value: the minimum distance to include
 * @param max_value: the maximum distance to include
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pathrough_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
	const char* field, float min_value, float max_value);

/*
 * Voxel filter.
 * @param cloud: point cloud object
 * @param lx: leafsize of x axis
 * @param ly: leafsize of y axis
 * @param lz: leafsize of z axis
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
	float lx, float ly, float lz);

/*
 * Statistical outliers removal.
 * @param cloud: point cloud object
 * @param n: how many points to be calculate mean in a neighbor
 * @param stddev: standard deviation 
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr statistical_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int n, double stddev);

/*
 * Radius outliers removal. If the number of points in a neighbor is samller than min_pts, the point will be removed.
 * @param cloud: point cloud object
 * @param radius: the value of radius of a ball neighbor
 * @param min_pts: the minimum number of points in a ball neighbor
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr radius_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double radius, int min_pts);

#endif // !__POINTCLOUD_h__
