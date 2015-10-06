/**
 * This is the defination woodbrick algorithm class 
 * @version 1.0.0
 * @author Yuhan Long <yuhanlon@andrew.cmu.edu>
 * All right reserved 
 */


#ifndef WOODBRICK_H_
#define WOODBRICK_H_

/* C++ STL library */
#include <iostream>
#include <assert.h>
#include <string>
#include <vector>
#include <cmath>

/* pcl basic */
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_ros/point_cloud.h>

/* pcl common */ 
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>

/* pcl sample_consensus */
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

/* pcl filter */
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>

/* algorithm base class */
#include "algorithm_template.h"


namespace evl
{
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT; 
    
    class GeometryAnalyzerWoodbrick : public AlgorithmTemplate <PointT>
    {
    /** 
     * Woodbrick class derived from base class
     */
    public:

        using AlgorithmTemplate<PointT>::cloud_in_;
        using AlgorithmTemplate<PointT>::click_points_;
        using AlgorithmTemplate<PointT>::robot_pose_;
        using AlgorithmTemplate<PointT>::cloud_out_;
        using AlgorithmTemplate<PointT>::model_coefficients_;
        using AlgorithmTemplate<PointT>::gripper_coefficients_;
        
        GeometryAnalyzerWoodbrick(const boost::shared_ptr<pcl::PointCloud <PointT> >, const boost::shared_ptr <std::vector<PointT> >,const boost::shared_ptr <std::vector<double> >);
        void compute();


    private:
        /**
         * private member:
         */ 
        PointCloudT::Ptr first_face;  /* face face on the woodbrick */
        PointCloudT::Ptr second_face; /* second face on the woodbrick*/
        uint64_t length;              /* length of woodbrick */
        uint64_t width;               /* width of woodbrick */
        uint64_t height;              /* height of woodbrick */
        PointT firstClick;            /* the first click on the wood brick */
        PointT secondClick;           /* the second click on the wood brick */
    };


}



#endif //WOODBRICK_H_
