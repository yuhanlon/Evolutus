/**
 * Implementation of the algorithm wrapper class 
 *
 * @Version 1.0.0
 * @Author Yuhan Long <yuhanlon@andrew.cmu.edu>
 * All right reserved
 */


#include "perception_base.h"


namespace evl
{
    template <typename PointT> 
    GeometryAnalyzerWrapper<PointT>::GeometryAnalyzerWrapper()
    {
        /** default constructor */
    }

    template <typename PointT> void
    GeometryAnalyzerWrapper<PointT>::compute()
    {   
        /**
         * when the wrapper compute method is called, the wrapper will first instantiate the algorithm according to the type
         * Then it will check and set the point cloud, click point and robot pose of the algorithm, and call the compute fucntion of the algorithm
         * After the compute function is run, the wrapper get the segmented point cloud, model coefficients and gripper postion (grasping stratgy)
         */

        ROS_INFO("[wrapper] compute entered");    
        
        if (!initAnalyzer(model_type_))
            PCL_ERROR("Error initializing the perception model");

        if (cloud_in_!=NULL)

        geo_->setInputCloud(cloud_in_);
        ROS_INFO("[wrapper] input cloud set");    
        geo_->setClickPoints(click_points_);
        ROS_INFO("[wrapper] click point set");    
        geo_->setRobotPose(robot_pose_);
        ROS_INFO("[wrapper] robot pose set");    
        geo_->compute();
        cloud_out_ = geo_->getSegClouds();
        model_coefficients_ = geo_->getModelCoefficients();
        gripper_coefficients_ = geo_->getGripperCoefficients();
    }


    template <typename PointT> bool 
    GeometryAnalyzerWrapper<PointT>::initAnalyzer(std::string &model_type_)
    {
        ROS_ERROR("[wrapper] initializing entered");    
        
        switch (model_type_[0])
        {
            case 'w'://OBJECT_WOOD_BRICK:
            {
            if (click_points_->size()<2) {
                //geo_.reset(new GeometryAnalyzerWoodbrick1(cloud_in_,click_points_,robot_pose_));
                geo_.reset(new GeometryAnalyzerWoodbrick1(cloud_in_,click_points_));
                ROS_ERROR("[wrapper] woodbrick 1 model initialized");    
             }
             else {    
                geo_.reset(new GeometryAnalyzerWoodbrick(cloud_in_,click_points_,robot_pose_));
                //geo_.reset(new GeometryAnalyzerWoodbrick(cloud_in_,click_points_));
                ROS_ERROR("[wrapper] woodbrick model initialized");    
             }
            break;
            }
            case 'v'://OBJECT_VALVE:
            {
            if (click_points_->size()<3) {
                geo_.reset(new GeometryAnalyzerValve(cloud_in_,click_points_));
                ROS_ERROR("[wrapper] valve model initialized");    
             }
             else {    
                geo_.reset(new GeometryAnalyzerValve3(cloud_in_,click_points_));
                ROS_ERROR("[wrapper] valve 3 model initialized");    
             }
            break;
            }
            case 'd':/** drill perception algorithm */
            {
            geo_.reset(new GeometryAnalyzerDrill(cloud_in_,click_points_,robot_pose_));
            ROS_ERROR("[wrapper] drill model initialized");    
            break;
            }
            default:
            {
                PCL_ERROR("No valid object type selected");
                return(false);
            }
            
        }

        return(true);
    }

template class GeometryAnalyzerWrapper<pcl::PointXYZ>;
}

