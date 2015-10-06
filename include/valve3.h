/**
 * This is the declaration of the valve class with three click points.
 * @version 1.0.0
 * @author Yuhan Long <yuhanlon@andrew.cmu.edu>
 * All right reserved 
 */

#ifndef VALVE3_H_
#define VALVE3_H_

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

/** pcl common */
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>

/** pcl sample_consensus */ 
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>


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
    
    class GeometryAnalyzerValve3 : public AlgorithmTemplate <PointT>
    {
        /**
         * This class is define for using three click algorithm in 
         */
    public:
        GeometryAnalyzerValve3(const boost::shared_ptr<pcl::PointCloud <PointT> >, const boost::shared_ptr <std::vector<PointT> >);
        void compute();


    protected:
        PointCloudT::Ptr cloud_ring;  /** point cloud of the ring on the valve */
        PointCloudT::Ptr cloud_spokes;  /** point cloud of the spokes on the valve */
        uint64_t spoke_num;  /** number of spokes on the valve */
        std::vector<double> spoke_angles;  /** angle of the spokes refer to the upright position */

        /** three click points */
        PointT firstClick;
        PointT secondClick;
        PointT thirdClick;
    };


}



#endif //VALVE3_H_
