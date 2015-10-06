/**
 * This is the declaration of the drill class.
 * @version 1.0.0
 * @author Yuhan Long <yuhanlon@andrew.cmu.edu>
 * All right reserved 
 */

#ifndef DRILL_H_
#define DRILL_H_

/** C++ standard template library */
#include <iostream>
#include <assert.h>
#include <string>
#include <vector>
#include <cmath>

/** pcl basic library */
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_ros/point_cloud.h>

/** pcl common/sample_consensus/filter library */
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/pca.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>

/** pcl MLS dependence */
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

/** pcl Euclidean Clustering dependence*/
#include <pcl/segmentation/extract_clusters.h>

// algorithm base class
#include "algorithm_template.h"

namespace evl
{
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT; 
    
    class GeometryAnalyzerDrill : public AlgorithmTemplate <PointT>
    {
        /**
         * class for drill perception algorithm. The algorihtm is model based 
         */ 
        GeometryAnalyzerDrill(const boost::shared_ptr<pcl::PointCloud <PointT> >, const boost::shared_ptr <std::vector<PointT> >, const boost::shared_ptr<std::vector<double> > robot_pose);
        void compute();


    protected:
        /** two click points on the drill */
        PointT firstClick;
        PointT secondClick;
    };


}



#endif //DRILL_H_
