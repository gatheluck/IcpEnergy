#ifndef ICP_ENERGY_HPP
#define ICP_ENERGY_HPP

#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "util.hpp"

/**
 * @fn
 * Calculate Jacobian of ICP energy term for the given source model point.
 * @param (引数名) 引数の説明
 * @param (引数名) 引数の説明
 * @detail 詳細な説明
 */
void calculate_icp_jacobian(const cv::Vec3f &model_vertex /* s */ ,
                            const cv::Vec3f &model_normal /* n */,
                            const cv::Vec3f &destination_vertex /* d */,
                            cv::Matx61f &J_icp)
{
  J_icp[0] = model_normal[0];
  J_icp[1] = model_normal[1];
  J_icp[2] = model_normal[2];
  J_icp[3] = (model_vertex[0]-destination_vertex[0])*(model_normal[1]-model_normal[2]) + model_normal[0]*(model_vertex[1]-model_vertex[2]);
  J_icp[4] = (model_vertex[1]-destination_vertex[1])*(model_normal[2]-model_normal[0]) + model_normal[1]*(model_vertex[2]-model_vertex[0]);
  J_icp[5] = (model_vertex[2]-destination_vertex[2])*(model_normal[0]-model_normal[1]) + model_normal[2]*(model_vertex[0]-model_vertex[1]);
}

#endif /* ICP_ENERGY_HP  */