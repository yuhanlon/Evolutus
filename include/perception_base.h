/** 
 * The wrapper function that provides a interface between the algorithm 'core' and the ROS interface.
 * @version 1.0.0
 * @author Yuhan Long <yuhanlon@andrew.cmu.edu>
 * ALl right reserved. 
 */

#ifndef PERCEPTION_BASE_H_
#define PERCEPTION_BASE_H_

/** STL include */
#include <string>
#include <vector>

/** PCL include */
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

/** Algorithm types include */
#include "algorithm_template.h"
#include "valve.h"
#include "valve3.h"
#include "woodbrick.h"
#include "woodbrick1.h"
#include "drill.h"

namespace evl 
{
    template <class PointT>        
    class GeometryAnalyzerWrapper
    {
        /**
         * This class provide a wrapper of the base algorithm class. This class is instantiate in the ROS subscriber routine. 
         * This wrapper will instantiate the algorithm base class as a place holder in the instantiate time. 
         * In the runtime, the subscriber routine will take UI selection of model type and instantiate the corresponding algorithm class.
         * Each functino and member in the algorithm base class has a wrapped correspondent here. 
         * This configuration gurantee the modularity of the perception algorithm group. Sample use:
         *  GeometryAnalyzerWrapper<Point T> perception;
         *  perception.setModelType("valve");
         *  perception.setCliclPoints(point_list);
         *  perception.setInputCloud(cloud);
         *  perception.compute()
         */
    public:

        GeometryAnalyzerWrapper();
        /** function wrapper for setting input point cloud */
        inline
        void setInputCloud(const boost::shared_ptr< pcl::PointCloud <PointT> > cloud_in) { cloud_in_  = cloud_in;}
        /** function wrapper for setting model type, model type include: valve, woodbrick, drill */
        inline 
        void setModelType(const std::string &model_type) { model_type_ = model_type;}
        /** function wrapper for setting the list of click points to the algorithm */
        inline
        void setClickPoints(const boost::shared_ptr<std::vector<PointT> > click_points) {click_points_ = click_points; }
        /** function wrapper for setting the robot current Pose {x,y,z,quat_a,quat_b,quat_r,quat_w} */ 
        inline
        void setRobotPose (const boost::shared_ptr <std::vector<double> > robot_pose) {robot_pose_ = robot_pose; }
        /** function wrapper for computing the model coefficient */
        void compute();
        /** function wrappper for getting the segmentation cloud */
        inline
        typename boost::shared_ptr <std::vector< pcl::PointCloud<PointT> > > getSegClouds() {return cloud_out_; }
        /** function wrapper for getting percept model coefficients */
        inline 
        boost::shared_ptr < std::vector<double> > getModelCoefficients() {return model_coefficients_; }
        /** function wrapper for getting the gripper pose */
        inline 
        boost::shared_ptr < std::vector<double> > getGripperCoefficients() {return gripper_coefficients_; }


    private:
        /** Private class member hide from other class */
        std::string model_type_;  /** model type */
        typename boost::shared_ptr<pcl::PointCloud<PointT> >  cloud_in_;  /** input point cloud for geometric analysis */
        typename boost::shared_ptr<std::vector<PointT> > click_points_;  /** click points list */
        typename boost::shared_ptr<std::vector< pcl::PointCloud<PointT> > > cloud_out_; /** segmented point clouds */
        boost::shared_ptr<std::vector<double> > model_coefficients_; /** pose of robot */
        boost::shared_ptr<std::vector<double> > gripper_coefficients_; /** detected object model */
        boost::shared_ptr<std::vector<double> > robot_pose_; /** grasping strategy */
        bool initAnalyzer(std::string &);  /** function to initalize different perection algorithm */

        typename boost::shared_ptr<evl::AlgorithmTemplate<PointT> > geo_;  /** pointer to the algorithm base class */
        
    };

}


#endif
