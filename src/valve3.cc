/**
 * This is the defination of the valve class with three click points.
 * @version 1.0.0
 * @author Yuhan Long <yuhanlon@andrew.cmu.edu>
 * All right reserved 
 */

#include "valve3.h"

namespace evl{
    GeometryAnalyzerValve3::GeometryAnalyzerValve3(
            const boost::shared_ptr< pcl::PointCloud <PointT> > cloud_in, 
            const boost::shared_ptr<std::vector<PointT> > click_points)
    {
        /** constructor function, setting click points, input point cloud and robot pose*/
        ROS_INFO("[valve3] Entering valve3 algorithm initiation");
        click_points_ = click_points;
        cloud_in_  = cloud_in;

        /** reinstantiate the model coefficients and gripper pose */
        ROS_INFO("[valve3] Preparing model coefficients reset");
        model_coefficients_.reset(new std::vector<double>);
        model_coefficients_->resize(12);
        gripper_coefficients_.reset(new std::vector<double>);
        gripper_coefficients_->resize(7);
        ROS_INFO("[valve3] Model coefficients reset");
        
        /** get the click point from the click point vector */
        assert(click_points_->size()>=3);
        firstClick = click_points->at(0);
        secondClick = click_points->at(1);
        thirdClick = click_points->at(2);
        ROS_INFO("[valve3] Click point set");
    }

    
    void GeometryAnalyzerValve3::compute()
    {
        /** compute function to calculate the model coefficents and gripper pose */

        PointCloudT::Ptr cloud_in (new PointCloudT);
        PointCloudT::Ptr cloud_temp (new PointCloudT);
        PointCloudT::Ptr cloud_valve (new PointCloudT);
        PointCloudT::Ptr cloud_valve_plane (new PointCloudT);
        PointCloudT::Ptr click_points (new PointCloudT);
        click_points->points.push_back(firstClick);    
        click_points->points.push_back(secondClick);    
        click_points->points.push_back(thirdClick); 

        /** estimate 3D ring from three click point */   
        pcl::SampleConsensusModelCircle3D<PointT>::Ptr click_ring_seg (new pcl::SampleConsensusModelCircle3D<PointT>(click_points));
        click_ring_seg -> setInputCloud(click_points);
        std::vector<int> samples;
        samples.resize(3);
        samples[0] = 0;
        samples[1] = 1;
        samples[2] = 2;
        cloud_in = cloud_in_;
        Eigen::VectorXf click_ring_coefficients;
        click_ring_seg -> computeModelCoefficients(samples,click_ring_coefficients);
        pcl::ModelCoefficients::Ptr coefficients_ring(new pcl::ModelCoefficients);
        coefficients_ring->values.resize(7);
    
        coefficients_ring->values[0]=click_ring_coefficients[0];
        coefficients_ring->values[1]=click_ring_coefficients[1];
        coefficients_ring->values[2]=click_ring_coefficients[2];
        coefficients_ring->values[3]=click_ring_coefficients[3];
        coefficients_ring->values[4]=click_ring_coefficients[4];
        coefficients_ring->values[5]=click_ring_coefficients[5];
        coefficients_ring->values[6]=click_ring_coefficients[6];
    
        pcl::SampleConsensusModelCircle3D<PointT>::Ptr ring_seg (new pcl::SampleConsensusModelCircle3D<PointT>(cloud_in));
        ring_seg -> setInputCloud(cloud_in);
        std::vector <int> ringIndices;
        pcl::IndicesPtr ringIndicesPtr (new std::vector<int> (ringIndices));
        ring_seg-> selectWithinDistance(click_ring_coefficients,0.02,*ringIndicesPtr);
        PointCloudT::Ptr cloud_ring(new PointCloudT(*cloud_in,*ringIndicesPtr));
        ROS_INFO("Number of points in the ring: %d\n",cloud_ring->size());

        /** pass the ring cloud into ransac again, estimate a set of more accurate result */
        pcl::SACSegmentation<PointT> seg_ring;
        seg_ring.setModelType(pcl::SACMODEL_CIRCLE3D);  /** change model to 3D ring */
        seg_ring.setMethodType(pcl::SAC_RANSAC);
        seg_ring.setInputCloud(cloud_ring);
        seg_ring.setMaxIterations(100);
        seg_ring.setDistanceThreshold(0.02);
        pcl::PointIndices::Ptr valve_plane (new pcl::PointIndices);
        seg_ring.segment(*valve_plane, *coefficients_ring);
        
        

        pcl::ExtractIndices<PointT> extract;
    
        PointCloudT::Ptr cloud_no_ring(new PointCloudT());
        extract.setInputCloud (cloud_in);
        extract.setIndices (ringIndicesPtr);
        extract.setNegative (true);
        extract.filter (*cloud_no_ring);
        
    
    
        pcl::SACSegmentation<PointT> seg;
        seg.setModelType(pcl::SACMODEL_PLANE);  /** change model to 3D ring */
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setInputCloud(cloud_ring);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.05);
        pcl::ModelCoefficients::Ptr coefficients_valve_plane(new pcl::ModelCoefficients);
        seg.segment(*valve_plane, *coefficients_valve_plane);
        
    
        Eigen::Vector4f valve_plane_vector;
        valve_plane_vector[0]=coefficients_valve_plane->values[0];
        valve_plane_vector[1]=coefficients_valve_plane->values[1];
        valve_plane_vector[2]=coefficients_valve_plane->values[2];
        valve_plane_vector[3]=coefficients_valve_plane->values[3];
        
        std::cout<<valve_plane_vector<<std::endl;
        pcl::SampleConsensusModelPlane<PointT>::Ptr ring_plane_seg (new pcl::SampleConsensusModelPlane<PointT>(cloud_in));
        ring_plane_seg -> setInputCloud(cloud_no_ring);
        std::vector <int> valvePlaneIndices;
        pcl::IndicesPtr valvePlaneIndicesPtr (new std::vector<int> (valvePlaneIndices));
        ring_plane_seg -> selectWithinDistance(valve_plane_vector,0.05,*valvePlaneIndicesPtr);
        cloud_no_ring.reset(new PointCloudT(*cloud_no_ring,*valvePlaneIndicesPtr));
       
    
        std::cout<<cloud_in->width<<std::endl;
        
        std::vector<float>::iterator iter;
        
    
        for(iter = coefficients_ring->values.begin();iter<=coefficients_ring->values.end();iter++) {
        std::cout<<*iter<<std::endl;
        }
        
    
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
        PointT center_point(coefficients_ring->values[0],coefficients_ring->values[1],coefficients_ring->values[2]);
        PointCloudT::Ptr cloud_spines (new PointCloudT());
        cloud_spines = cloud_no_ring;
    
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
              seg.setInputCloud(cloud_spines);
              seg.setModelType(pcl::SACMODEL_LINE);
              seg.setMethodType(pcl::SAC_RANSAC);
              seg.setMaxIterations(20);
              seg.setDistanceThreshold(0.03);
              seg.segment(*inliers_plane, *coefficients_spine);
              Eigen::Vector4f centroid_point;
              pcl::compute3DCentroid(*cloud_spines, *inliers_plane,centroid_point);
    
              float distance = 0;
              for (int i=0;i<3;i++)
              distance += (centroid_point[i]-coefficients_ring->values[i])*(centroid_point[i]-coefficients_ring->values[i]);
              distance = sqrt(distance);
              if (distance >= coefficients_ring->values[3]*0.5) {
              
              /** the spine is single direction */
              for(int i = 0; i<3 ; i++)
              coefficients_spine->values[i+3]=((centroid_point[i]-coefficients_ring->values[i])*coefficients_spine->values[i+3]<0) ? -coefficients_spine->values[i+3] : coefficients_spine->values[i+3];
     
              spinesVector.push_back(Eigen::Vector3f(coefficients_spine->values[3],coefficients_spine->values[4],coefficients_spine->values[5])); 
              }
              else
              {
              /** the spine is double direction */
              spinesVector.push_back(Eigen::Vector3f(coefficients_spine->values[3],coefficients_spine->values[4],coefficients_spine->values[5])); 
              spinesVector.push_back(Eigen::Vector3f(-coefficients_spine->values[3],-coefficients_spine->values[4],-coefficients_spine->values[5])); 
              }
    
              extract.setInputCloud(cloud_spines);
              extract.setIndices(inliers_plane);
              extract.setNegative(true);
              extract.filter(*cloud_spines_rest);
    
              cloud_spines = cloud_spines_rest;
              sor.setInputCloud(cloud_spines);
              sor.setMeanK(8);
              sor.setStddevMulThresh(1);
              sor.filter(*cloud_spines_rest);
              cloud_spines = cloud_spines_rest;
         }
        std::vector<Eigen::Vector3f>::iterator spinesIter;
        assert(spinesVector.size()!=0);
        //
        std::cout << "========== Spine detection ==========" << std::endl;
        std::cout << "Number of Spines: "<< spinesVector.size()<<std::endl;
        //
        Eigen::Vector3f planeToValve(center_point.x-firstClick.x,center_point.y-firstClick.y,center_point.z-firstClick.z);
        Eigen::Vector3f planeNormal(coefficients_ring->values[4],coefficients_ring->values[5],coefficients_ring->values[6]);
        if(planeToValve.dot(planeNormal)<0) {
        planeNormal = -planeNormal;
        }/** make sure the plane Normal is point to from the wall to the valve */
        
        Eigen::Vector3f referenceVector(0.0f,0.0f,1.0f);
        Eigen::Vector3f rightVector = referenceVector.cross(planeNormal);
        
        
        for (spinesIter = spinesVector.begin(); spinesIter != spinesVector.end();spinesIter++) {
           float degree=pcl::rad2deg(acos((*spinesIter).dot(referenceVector)/(sqrt(referenceVector.dot(referenceVector))*((*spinesIter).dot(*spinesIter)))));
           if(rightVector.dot(*spinesIter)<0)
               degree=360-degree;
              std::cout << "Degree: "<< degree<<std::endl;
              std::cout << "========== Spine detection ==========" << std::endl;
              std::cout << *spinesIter << std::endl;
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
        this->gripper_coefficients_->at(2) = coefficients_ring -> values[2]+coefficients_ring -> values[3]+0.05f;
        this->gripper_coefficients_->at(3) = quat_palm.w();
        this->gripper_coefficients_->at(4) = quat_palm.x();
        this->gripper_coefficients_->at(5) = quat_palm.y();
        this->gripper_coefficients_->at(6) = quat_palm.z();


        }
    }
}
