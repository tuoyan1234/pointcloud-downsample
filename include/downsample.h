/*
 * @Author: tyz 1872516355@qq.com
 * @Date: 2026-04-15 18:57:05
 * @LastEditors: tyz 1872516355@qq.com
 * @LastEditTime: 2026-04-15 19:00:17
 * @FilePath: /Desktop/点云降采样/include/downsample.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _DOWN_SAMPLE_H_
#define _DOWN_SAMPLE_H_

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
// 点云可视化
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <yaml-cpp/yaml.h>

#include <iostream>

class PointscloudDownSample{
public:
    PointscloudDownSample();
    ~PointscloudDownSample();
    bool randomDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float leaf_size);
    bool voxelGridDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float leaf_size);
    bool passThroughDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, std::string axis, float min, float max);
    bool uniformSamplingDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float leaf_size);
    // 读取配置文件
    bool readConfig(std::string configFile,PointscloudDownSample& downsample);
public:
    bool isPassthroughdownsample = false;
    bool isRandomdownsample  = false;
    bool isVoxeldownSample   = false;
    bool isUniformdownsample = false;
    std::string passthroughAxis = "x";
    float passthroughMin = 0.0;
    float passthroughMax = 0.0;
    float leafSize = 0.1;
    std::string plyfilePath = "data/pointcloud.ply";
};

#endif // !_DOWN_SAMPLE_H_
