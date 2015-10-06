/**
 * This is the one click point woodbrick algorithm class 
 * @version 1.0.0
 * @author Yuhan Long <yuhanlon@andrew.cmu.edu>
 * All right reserved 
 */

#ifndef WOODBRICK1_H_
#define WOODBRICK1_H_

/** C++ COMMON INCLUDE */
#include <iostream>
#include <assert.h>
#include <string>
#include <vector>
#include <cmath>
/** pcl basic */
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_ros/point_cloud.h>

/** pcl common */ 
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>

/** pcl sample_consensus */
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

/** pcl filter */
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>


#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>

/** algorithm base class */
#include "algorithm_template.h"


namespace evl
{
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT; 
    
    class GeometryAnalyzerWoodbrick1 : public AlgorithmTemplate <PointT>
    {
    /** 
     * One click point woodbrick class derived from base class
     */
    public:

        using AlgorithmTemplate<PointT>::cloud_in_;
        using AlgorithmTemplate<PointT>::click_points_;
        using AlgorithmTemplate<PointT>::cloud_out_;
        using AlgorithmTemplate<PointT>::robot_pose_;
        using AlgorithmTemplate<PointT>::model_coefficients_;
        using AlgorithmTemplate<PointT>::gripper_coefficients_;
        
        GeometryAnalyzerWoodbrick1(const boost::shared_ptr<pcl::PointCloud <PointT> >, const boost::shared_ptr <std::vector<PointT> >);
        void compute();


    private:

        /**
         * private member:
         */ 
        PointCloudT::Ptr first_face;  /** first face on wood brick */
        PointCloudT::Ptr second_face;  /** second face on wood brick */
        uint64_t length; /** length of the wood brick */
        uint64_t width;  /** width of the wood brick */
        uint64_t height;  /** height of the wood brick */
        PointT firstClick;  /** click point on the object plane */

        float unitVectorAngle(float, float, float, float, float, float); /** unify the angle of vectors */
        float distL2(float, float, float, float, float, float);  /** calculate distance between vectors */

    };
}



#endif //WOODBRICK1_H_
