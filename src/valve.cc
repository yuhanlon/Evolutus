/**
 * This is the defination of the valve class.
 * @version 1.0.0
 * @author Yuhan Long <yuhanlon@andrew.cmu.edu>
 * All right reserved 
 */


#include "valve.h"

namespace evl{
    GeometryAnalyzerValve::GeometryAnalyzerValve(
            const boost::shared_ptr< pcl::PointCloud <PointT> > cloud_in, 
            const boost::shared_ptr<std::vector<PointT> > click_points)
    {
        /** constructor function, setting click points, input point cloud and robot pose*/
        ROS_INFO("[valve] Entering valve algorithm initiation");
        click_points_ = click_points;
        cloud_in_  = cloud_in;
        ROS_INFO("[valve] Preparing model coefficients reset");

         /** reinstantiate the model coefficients and gripper pose */
        model_coefficients_.reset(new std::vector<double>);
        model_coefficients_->resize(12);
        gripper_coefficients_.reset(new std::vector<double>);
        gripper_coefficients_->resize(7);
        ROS_INFO("[valve] Model coefficients reset");
        
        /** get the click point from the click point vector */
        firstClick = click_points->at(0);
        ROS_INFO("[valve] Click point set");
    }

    
    void GeometryAnalyzerValve::compute()
    {
        /** compute function to calculate the model coefficents and gripper pose */
        PointCloudT::Ptr cloud_in (new PointCloudT);
        PointCloudT::Ptr cloud_temp (new PointCloudT);
        PointCloudT::Ptr cloud_valve (new PointCloudT);
        PointCloudT::Ptr cloud_valve_plane (new PointCloudT);
        PointCloudT::Ptr cloud_ring (new PointCloudT);
        
    
        cloud_in = this->cloud_in_;
    
         /** Segment the wall by finding the largest plane in the point cloud */
        pcl::ModelCoefficients::Ptr coefficients_wall(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
        pcl::SACSegmentation<PointT> seg;
        pcl::ExtractIndices<PointT> extract;
        
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.05);
    
        seg.setInputCloud(cloud_in);
     
    
        assert(cloud_in->width > 0);
    
        seg.segment(*inliers_plane, *coefficients_wall);
        
    
        PointT plane_point = (cloud_in->points)[(inliers_plane->indices)[1]];  /** define a point of on the plane for future vector calculation */
        
    
        extract.setInputCloud(cloud_in);
        extract.setIndices(inliers_plane);
        extract.setNegative(true);
        extract.filter(*cloud_valve_plane); /** extract the point cloud without the valve.*/
    
        /** wall removal finished */
        
        
        
        /** estimate the points on the valve */
        /** first, calculate the plnane of the valve, by the click point and the normal of the wall */
        Eigen::Vector4f valve_plane_coefficient;
        valve_plane_coefficient[0]=coefficients_wall->values[0];
        valve_plane_coefficient[1]=coefficients_wall->values[1];
        valve_plane_coefficient[2]=coefficients_wall->values[2];
        valve_plane_coefficient[3]=-coefficients_wall->values[0]*firstClick.x-coefficients_wall->values[1]*firstClick.y-coefficients_wall->values[2]*firstClick.z;
        
    
        /** Second, according to the valve plane coefficients, get points of the valve plane */ 
        pcl::SampleConsensusModelPlane<PointT>::Ptr valvePlane (new pcl::SampleConsensusModelPlane<PointT>(cloud_valve_plane));
        std::vector <int> valvePlaneIndices;
        pcl::IndicesPtr valvePlaneIndicesPtr (new std::vector<int> (valvePlaneIndices));
        
        valvePlane -> setInputCloud(cloud_valve_plane);
        
        assert(cloud_valve_plane -> width > 0);
    
        valvePlane-> selectWithinDistance(valve_plane_coefficient,0.03,*valvePlaneIndicesPtr);
        PointCloudT::Ptr valve_plane_extract(new PointCloudT(*cloud_valve_plane,*valvePlaneIndicesPtr));
        cloud_valve_plane = valve_plane_extract;
        
    
        /** Third, estimate 3D ring on the valve plane */
        pcl::ModelCoefficients::Ptr coefficients_ring(new pcl::ModelCoefficients);
        PointCloudT::Ptr cloud_no_ring (new PointCloudT);
       
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CIRCLE3D); /** change model to 3D ring */
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setInputCloud(cloud_valve_plane);
        assert(cloud_valve_plane->width > 0);
        seg.segment(*inliers_plane, *coefficients_ring);
        
        
        /** OPTIONAL! output the coefficients */
        std::vector<float>::iterator iter;
        for(iter = coefficients_ring->values.begin();iter<=coefficients_ring->values.end();iter++) {
        std::cout<<*iter<<std::endl;
        }
        PointT center_point(coefficients_ring->values[0],coefficients_ring->values[1],coefficients_ring->values[2]); /** record the center point of the ring */
       
        /** extract the ring point cloud  */
        extract.setInputCloud(cloud_valve_plane);
        extract.setIndices(inliers_plane);
        extract.setNegative(false);
        extract.filter(*cloud_ring);
        extract.setNegative(true);
        extract.filter(*cloud_no_ring); /** the point cloud without ring, NOT THE SPINE POINT CLOUD! */
    
    
    
        /** extract spine points from the cloud_no_ring point cloud according to the valve plane coefficients.*/
       
        pcl::ModelCoefficients::Ptr coefficients_valve_plane(new pcl::ModelCoefficients);
        coefficients_valve_plane->values.resize(4);
        coefficients_valve_plane->values[0] = valve_plane_coefficient[0];
        coefficients_valve_plane->values[1] = valve_plane_coefficient[1];
        coefficients_valve_plane->values[2] = valve_plane_coefficient[2];
        coefficients_valve_plane->values[3] = valve_plane_coefficient[3];
        
        
        pcl::SampleConsensusModelPlane<PointT>::Ptr valvePlaneSeg (new pcl::SampleConsensusModelPlane<PointT> (cloud_no_ring));
        valvePlaneSeg -> setInputCloud(cloud_no_ring);
        assert(cloud_no_ring->width > 0);
        
        std::vector <int> spinesIndices;
        valvePlaneSeg -> selectWithinDistance(valve_plane_coefficient, 0.03,spinesIndices);
        PointCloudT::Ptr cloud_spines (new PointCloudT(*cloud_no_ring,spinesIndices));
        
    
    
        /** projected the point cloud of the spines to the valve plane, increasing the angel estimation accuracy */
        pcl::ProjectInliers<PointT> proj_spines;
        proj_spines.setModelType(pcl::SACMODEL_PLANE);
        proj_spines.setInputCloud(cloud_spines);
        proj_spines.setModelCoefficients(coefficients_valve_plane);
        PointCloudT::Ptr cloud_projected (new PointCloudT);
        proj_spines.filter(*cloud_projected);
        cloud_spines = cloud_projected;
        
        
        /** detect the angle */
        pcl::StatisticalOutlierRemoval<PointT> sor (true);
        int cloudSpinesNumber = cloud_spines -> size();
        std::vector<Eigen::Vector3f> spinesVector; 
        PointCloudT::Ptr cloud_spines_rest (new PointCloudT);    
        PointCloudT::Ptr cloud_spines_inliers (new PointCloudT);    
        pcl::ModelCoefficients::Ptr coefficients_spine(new pcl::ModelCoefficients);
     
        
        while(cloud_spines->size()>=cloudSpinesNumber/10) {
        
            /** first find a 3D line in the spines point cloud */
            seg.setInputCloud(cloud_spines);
            seg.setModelType(pcl::SACMODEL_LINE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(20);
            seg.setDistanceThreshold(0.05);
            seg.segment(*inliers_plane, *coefficients_spine);
            
            assert(coefficients_spine->values.size()==6);
    
    
            
            Eigen::Vector4f centroid_point;
            pcl::compute3DCentroid(*cloud_spines, *inliers_plane,centroid_point);
            
            /** calculate the distance between the center of the line and the center of the ring */
            float distance = 0;
            for (int i=0;i<3;i++) distance += (centroid_point[i]-coefficients_ring->values[i])*(centroid_point[i]-coefficients_ring->values[i]);
            distance = sqrt(distance);
            
    
            if (distance >= coefficients_ring->values[3]*0.4) {
             /** the spine is single direction */
                for(int i = 0; i<3 ; i++)
                coefficients_spine->values[i+3]=((centroid_point[i]-coefficients_ring->values[i])*coefficients_spine->values[i+3]<0) ? -coefficients_spine->values[i+3] : coefficients_spine->values[i+3]; //rectify the vector to point outward
                spinesVector.push_back(Eigen::Vector3f(coefficients_spine->values[3],coefficients_spine->values[4],coefficients_spine->values[5])); 
            }
            else
            {
            /** the spine is double direction */
            spinesVector.push_back(Eigen::Vector3f(coefficients_spine->values[3],coefficients_spine->values[4],coefficients_spine->values[5])); 
            spinesVector.push_back(Eigen::Vector3f(-coefficients_spine->values[3],-coefficients_spine->values[4],-coefficients_spine->values[5])); 
                
            }
            
            /** remove the spine that are already segmented */
            extract.setInputCloud(cloud_spines);
            extract.setIndices(inliers_plane);
            extract.setNegative(true);
            extract.filter(*cloud_spines_rest);
            cloud_spines = cloud_spines_rest;
            
            /** filtering noises points */
            sor.setInputCloud(cloud_spines);
            sor.setMeanK(8);
            sor.setStddevMulThresh(0.3);
            sor.filter(*cloud_spines_rest);
            cloud_spines = cloud_spines_rest;
        }
    
    
        std::vector<Eigen::Vector3f>::iterator spinesIter;
        assert(spinesVector.size()!=0);
        
        std::cout << "========== Spine detection ==========" << std::endl;
        std::cout << "Number of Spines: "<< spinesVector.size()<<std::endl;
    
        /** calculating the angle of the spines, reference to the vector (0,0,1) 
         * HERE WE ASSUME THE VALVE IS PERPENCULAR TO THE GROUNG
         * NOT ALWAYS TRUE!!*/
    
        Eigen::Vector3f planeToValve(center_point.x-plane_point.x,center_point.y-plane_point.y,center_point.z-plane_point.z);
        Eigen::Vector3f planeNormal(coefficients_ring->values[4],coefficients_ring->values[5],coefficients_ring->values[6]);
        if(planeToValve.dot(planeNormal)<0) {
        planeNormal = -planeNormal;
        }/** make sure the plane Normal is point to from the wall to the valve */
        
        Eigen::Vector3f referenceVector(0.0f,0.0f,1.0f);
        Eigen::Vector3f rightVector = referenceVector.cross(planeNormal);
        std::vector<double> spoke_angles;
        
        for (spinesIter = spinesVector.begin(); spinesIter != spinesVector.end();spinesIter++) {
            double degree=pcl::rad2deg(acos((*spinesIter).dot(referenceVector)/(sqrt(referenceVector.dot(referenceVector))*((*spinesIter).dot(*spinesIter)))));
            if(rightVector.dot(*spinesIter)<0) degree=360-degree;
            spoke_angles.push_back(degree);
            std::cout << "Degree: "<< degree<<std::endl;
        }
    

        rightVector = -rightVector.normalized();
        Eigen::Matrix3f rot;
        rot(0,0) = referenceVector[0];
        rot(1,0) = referenceVector[1];
        rot(2,0) = referenceVector[2];

        rot(0,1) = rightVector[0];
        rot(1,1) = rightVector[1];
        rot(2,1) = rightVector[2];

        rot(0,2) = planeNormal[0];
        rot(1,2) = planeNormal[1];
        rot(2,2) = planeNormal[2];    

        Eigen::Quaternion<float> quat(rot); 

        ROS_INFO_STREAM(quat.x() << quat.y() << quat.z() << quat.w());

        this->cloud_ring = cloud_ring;
        this->cloud_spokes = cloud_projected;
        this->model_coefficients_->push_back(1.0);        
        this->model_coefficients_->at(1) = coefficients_ring -> values[0];
        this->model_coefficients_->at(2) = coefficients_ring -> values[1];
        this->model_coefficients_->at(3) = coefficients_ring -> values[2];
        this->model_coefficients_->at(4) = coefficients_ring -> values[3];
        this->model_coefficients_->at(5) = coefficients_ring -> values[4];
        this->model_coefficients_->at(6) = coefficients_ring -> values[5];
        this->model_coefficients_->at(7) = coefficients_ring -> values[6];
        this->model_coefficients_->at(8) = quat.w();
        this->model_coefficients_->at(9) = quat.x();
        this->model_coefficients_->at(10) = quat.y();
        this->model_coefficients_->at(11) = quat.z();

        
       /** set the output fields */
        this->spoke_num = spinesVector.size();
        this->model_coefficients_->push_back(spinesVector.size());
        this->spoke_angles.resize(spoke_num); 
        for (int i = 0; i < spoke_num; i++) {
        this->model_coefficients_->push_back(spoke_angles[i]);
        this->spoke_angles[i] = spoke_angles[i];

        Eigen::Matrix3f rot_palm;

        rot_palm(0,1) = referenceVector[0];
        rot_palm(1,1) = referenceVector[1];
        rot_palm(2,1) = -referenceVector[2];

        rot_palm(0,2) = -rightVector[0];
        rot_palm(1,2) = -rightVector[1];
        rot_palm(2,2) = -rightVector[2];

        rot_palm(0,0) = planeNormal[0];
        rot_palm(1,0) = planeNormal[1];
        rot_palm(2,0) = planeNormal[2];    

        Eigen::Quaternion<float> quat_palm(rot_palm); 

        this->gripper_coefficients_->at(0) = coefficients_ring -> values[0];
        this->gripper_coefficients_->at(1) = coefficients_ring -> values[1];
        this->gripper_coefficients_->at(2) = coefficients_ring -> values[2]+coefficients_ring -> values[3];
        this->gripper_coefficients_->at(3) = quat_palm.w();
        this->gripper_coefficients_->at(4) = quat_palm.x();
        this->gripper_coefficients_->at(5) = quat_palm.y();
        this->gripper_coefficients_->at(6) = quat_palm.z();


        }
    }
}
