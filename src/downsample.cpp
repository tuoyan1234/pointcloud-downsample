/*
 * @Author: tyz 1872516355@qq.com
 * @Date: 2026-04-15 18:57:12
 * @LastEditors: tyz 1872516355@qq.com
 * @LastEditTime: 2026-04-15 19:04:49
 * @FilePath: /Desktop/点云降采样/src/downsample.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "downsample.h"

PointscloudDownSample::PointscloudDownSample(){}

PointscloudDownSample::~PointscloudDownSample(){}

bool PointscloudDownSample::randomDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float leaf_size){
    if(cloud_in->points.size() == 0){
        std::cout << "randomDownSample, Input cloud is empty!" << std::endl;
        return false;
    }
    pcl::RandomSample<pcl::PointXYZ> random_sample;
    random_sample.setInputCloud(cloud_in);
    random_sample.setSample(100);
    random_sample.filter(*cloud_out);

    std::cout << "randomDownSample success!" << std::endl;
    return true;
}

bool PointscloudDownSample::voxelGridDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float leaf_size){
    if(cloud_in->points.size() == 0){
        std::cout << "voxelGridDownSample, Input cloud is empty!" << std::endl;
        return false;
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud_in);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*cloud_out);

    std::cout << "voxelGridDownSample success!" << std::endl;
    return true;
}

bool PointscloudDownSample::passThroughDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, std::string axis, float min, float max){
    if(cloud_in->points.size() == 0){
        std::cout << "passThroughDownSample, Input cloud is empty!" << std::endl;
        return false;
    }
    pcl::PassThrough<pcl::PointXYZ> pass_through;
    pass_through.setInputCloud(cloud_in);
    pass_through.setFilterFieldName(axis);
    pass_through.setFilterLimits(min, max);
    pass_through.filter(*cloud_out);

    std::cout << "passThroughDownSample, success!" << std::endl;
    return true;
}

bool PointscloudDownSample::uniformSamplingDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float leaf_size){
    if(cloud_in->points.size() == 0){
        std::cout << "uniformSamplingDownSample, Input cloud is empty!" << std::endl;
        return false;
    }
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud(cloud_in);
    uniform_sampling.setRadiusSearch(leaf_size);
    uniform_sampling.filter(*cloud_out);

    std::cout << "uniformSamplingDownSample, success!" << std::endl;
    return true;
}

bool PointscloudDownSample::readConfig(std::string configFile, PointscloudDownSample& downsample){
    YAML::Node config = YAML::LoadFile(configFile);

    // isPassthroughdownsample
    if(config["downsampleParams"]["isPassthroughdownsample"].IsDefined()){
        downsample.isPassthroughdownsample = config["downsampleParams"]["isPassthroughdownsample"].as<bool>();
        downsample.passthroughMin = config["downsampleParams"]["passthroughMin"].as<float>();
        downsample.passthroughMax = config["downsampleParams"]["passthroughMax"].as<float>();
        if(downsample.isPassthroughdownsample){
            std::cout << "isPassthroughdownsample, passthroughMin: " << passthroughMin << ", passthroughMax: " << passthroughMax << std::endl;
        }
    }else{
        std::cout << "isPassthroughdownsample not defined" << std::endl;
    }

    // isRandomdownsample
    if(config["downsampleParams"]["isRandomdownsample"].IsDefined()){
        downsample.isRandomdownsample = config["downsampleParams"]["isRandomdownsample"].as<bool>();
        if(downsample.isRandomdownsample){
            std::cout << "start Randomdownsample" << std::endl;
        }
    }else{
        std::cout << "isRandomdownsample not defined" << std::endl;
    }

    // isVoxeldownSample
    if(config["downsampleParams"]["isVoxeldownSample"].IsDefined()){
        downsample.isVoxeldownSample = config["downsampleParams"]["isVoxeldownSample"].as<bool>();
        if(downsample.isVoxeldownSample){
            std::cout << "start VoxeldownSample" << std::endl;
        }
    }else{
        std::cout << "isVoxeldownSample not defined" << std::endl;
    }

    // isUniformdownsample
    if(config["downsampleParams"]["isUniformdownsample"].IsDefined()){
        downsample.isUniformdownsample = config["downsampleParams"]["isUniformdownsample"].as<bool>();
        if(downsample.isUniformdownsample){
            std::cout << "start Uniformdownsample" << std::endl;
        }
    }else{
        std::cout << "isUniformdownsample not defined" << std::endl;
    }

    // ply-path
    if(config["plyFilePath"].IsDefined()){
        downsample.plyfilePath = config["plyFilePath"].as<std::string>();
        std::cout << "plyFilePath: " << downsample.plyfilePath << std::endl;
    }else{
        std::cout << "plyFilePath not defined" << std::endl;
    }

    // passthroughAxis
    if(config["downsampleParams"]["passthroughAxis"].IsDefined()){
        downsample.passthroughAxis = config["downsampleParams"]["passthroughAxis"].as<std::string>();
        std::cout << "passthroughAxis: " << passthroughAxis << std::endl;
    }else{
        std::cout << "passthroughAxis not defined" << std::endl;
    }

    // leafSize
    if(config["downsampleParams"]["leafSize"].IsDefined()){
        downsample.leafSize = config["downsampleParams"]["leafSize"].as<float>();
        if(downsample.leafSize){
            std::cout << "leafSize: " + std::to_string(downsample.leafSize) << std::endl;
        }
    }else{
        std::cout << "leafSize not defined" << std::endl;
    }

    return true;
}
