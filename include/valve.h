/**
 * This is the declaration of the valve class.
 * @version 1.0.0
 * @author Yuhan Long <yuhanlon@andrew.cmu.edu>
 * All right reserved 
 */

#ifndef VALVE_H_
#define VALVE_H_

/** C++ common library */
#include <iostream>
#include <assert.h>
#include <string>
#include <vector>
#include <cmath>


/** pcl library */
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_ros/point_cloud.h>

/** pcl common include */
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>

/** pcl sample_consensus */
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>

/** pcl filter */
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>

/** algorithm base class */
#include "algorithm_template.h"


namespace evl
{
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT; 
    
    class GeometryAnalyzerValve : public AlgorithmTemplate <PointT>
    {
        /**
         * class for valve geometry analysis, this class derived from the base algorithm class
         */
    public:    
        GeometryAnalyzerValve(const boost::shared_ptr<pcl::PointCloud <PointT> >, const boost::shared_ptr <std::vector<PointT> >);
        void compute();


    private:
        /** private member */
        PointCloudT::Ptr cloud_ring; /** the point cloud of the ring on the valve */
        PointCloudT::Ptr cloud_spokes; /** the point cloud of the spoke on the valve */
        uint64_t spoke_num;  /** number of spokes in the image */
        std::vector<double> spoke_angles;  /** list of the angle of the spoke refer to a upright direction */
        PointT firstClick; /** the first click point */
    };


}



#endif //VALVE_H_
