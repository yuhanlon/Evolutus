/**
 * Provide a base class(template) for different algorithm. 
 * This allows the program dynamically decides the algorithm in run time.
 * @version 1.0.0
 * @author Yuhan Long <yuhanlon@andrew.cmu.edu>
 * All right reserved 
 */



#ifndef ALGORITHM_TEMPLATE_H_
#define ALGORITHM_TEMPLATE_H_

/** STL include */
#include <string>
#include <assert.h>
#include <string>


/** PCL include */
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>


namespace evl
{
    template <class PointT>
    class AlgorithmTemplate 
    {
    /**
     * This class can be use as a template for object perception algorithm. The calling stack of classes is:
     * GeometryAnalyzerWrapper --> AlgorithmTemplate --> Valve
     *                                               |
     *                                               --> Valve3
     *                                               |
     *                                               --> woodbrick
     *                                               |
     *                                               --> woodbrick1   
     *                                               |
     *                                               --> drill
     * Sample usage:
     * boost::shared_ptr <AlgorithmTemplate>  geo_;
     * geo_.reset(new GeometryAnalyzerValve(cloud_in_,click_points_);
     * geo_.compute();
     */ 



    public:
        /** Rechieve point cloud from ROS interface*/
        inline 
        void setInputCloud (const boost::shared_ptr <pcl::PointCloud <PointT> > cloud_in) { cloud_in_ = cloud_in; };
        /** Get a vector of click points from ROS interface */
        inline
        void setClickPoints (const boost::shared_ptr <std::vector<PointT> > click_points) {click_points_ = click_points; };
        
        inline 
        void setModelType(const std::string &model_type) { model_type_ = model_type;};
        /** Get the robot current Pose {x,y,z,quat_a,quat_b,quat_r,quat_w} */ 
        inline
        void setRobotPose (const boost::shared_ptr <std::vector<double> > robot_pose) {robot_pose_ = robot_pose; };
        /** Virtual function for computing, each derived algorithm class should have its own compute function */
        inline
        virtual void compute(){}
        /** Return segmented point clouds to ROS interface, point cloud types depend on the selected model */
        inline 
        typename boost::shared_ptr <std::vector< pcl::PointCloud<PointT> > > getSegClouds() {return cloud_out_; };
        /** Return percepted model coefficents to ROS interface, coefficents format depend on the selected model */
        inline 
        boost::shared_ptr <std::vector <double> > getModelCoefficients() {return model_coefficients_; };
        /** Return possible grasping strategy to ROS interface, Gripper Pose is encoded as {x,y,z,quat_a,quat_b,quat_r,quat_w */
        inline 
        boost::shared_ptr <std::vector <double> > getGripperCoefficients() {return gripper_coefficients_; };
   
    protected:
        /** Protected class members for algorithm usage
          * These members can only be accessed by algorithm class
          */
        std::string model_type_;  /** model type name */
        typename boost::shared_ptr< pcl::PointCloud<PointT> > cloud_in_;  /** point cloud to be process */
        typename boost::shared_ptr< std::vector<PointT> > click_points_;  /** click point from users */
        typename boost::shared_ptr< std::vector<pcl::PointCloud<PointT> > > cloud_out_; /** segmented point clouds */
        boost::shared_ptr< std::vector<double> > robot_pose_;  /** pose of robot */
        boost::shared_ptr< std::vector<double> > model_coefficients_;  /** detected object model */
        boost::shared_ptr< std::vector<double> > gripper_coefficients_;  /** grasping strategy */
        


    }; 
}

#endif 
