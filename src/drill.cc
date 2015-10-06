/**
 * This is the defination of the drill class.
 * @version 1.0.0
 * @author Yuhan Long <yuhanlon@andrew.cmu.edu>
 * All right reserved 
 */

#include "drill.h"

namespace evl{
    GeometryAnalyzerDrill::GeometryAnalyzerDrill(
            const boost::shared_ptr< pcl::PointCloud <PointT> > cloud_in, 
            const boost::shared_ptr<std::vector<PointT> > click_points,
            const boost::shared_ptr<std::vector<double> > robot_pose)
    {
        /** constructor function, setting click points, input point cloud and robot pose*/
        ROS_INFO("[drill] Entering drill algorithm initiation");
        click_points_ = click_points;
        cloud_in_  = cloud_in;
        robot_pose_ = robot_pose;
        
        /** reinstantiate the model coefficients and gripper pose */
        ROS_INFO("[drill] Preparing model coefficients reset");
        model_coefficients_.reset(new std::vector<double>);
        model_coefficients_->resize(12);
        gripper_coefficients_.reset(new std::vector<double>);
        gripper_coefficients_->resize(7);
        cloud_out_.reset(new std::vector<pcl::PointCloud<PointT> >);
        cloud_out_->resize(1);

        /** get the click point from the click point vector */
        ROS_INFO("[drill] Model coefficients reset");
        firstClick = click_points->at(0);
        secondClick = click_points->at(1);
        ROS_INFO("[drill] Click point set");
    }

    
    void GeometryAnalyzerDrill::compute()
    {
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_model(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_model_in(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_object(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_mid(new pcl::PointCloud<pcl::PointNormal>);
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d transformation_matrix_centroid = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d transformation_matrix_pca = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d transformation_matrix_icp = Eigen::Matrix4d::Identity();
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_model_normal (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_object_normal (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in_normal (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_mid_normal (new pcl::PointCloud<pcl::PointNormal>);
    


    Eigen::Vector4f centroid_model;
    Eigen::Vector4f centroid_object;
    Eigen::Vector4f distance_centroid;

    double robot_x;
    double robot_y;
    double robot_z;
    if (robot_pose_->size()==3)
    {
        robot_x = robot_pose_->at(0);
        robot_y = robot_pose_->at(1);
        robot_z = robot_pose_->at(2);
    }
    else
    {
        robot_x = 0;
        robot_y = 0;
        robot_z = 0;
    }

    
    PointT midClickPoint((firstClick.x+secondClick.x)/2,(firstClick.y+secondClick.y)/2,(firstClick.z+secondClick.z)/2);
    Eigen::Vector4f maxPoint(0.2, 0.05,0.2,1);
    Eigen::Vector4f minPoint(-0.2,-0.05,-0.3,1); 
    Eigen::Vector3f trans_box(midClickPoint.x, midClickPoint.y, midClickPoint.z);
    
    /** Create a bounding box to segment the drill model roughly */
    pcl::CropBox<PointT> box;
    box.setInputCloud (cloud_in_);
    box.setMin(minPoint);
    box.setMax(maxPoint);
    box.setTranslation(trans_box);
    
    Eigen::Vector3f reference_x(1.0,0.0,0.0);
    Eigen::Vector3f drill_l(secondClick.x-firstClick.x,secondClick.y-firstClick.y,0.0); 
    drill_l.normalize();
    float cosRtoD = reference_x.dot(drill_l)/(sqrt(reference_x.dot(reference_x))*sqrt(drill_l.dot(drill_l)));
    Eigen::Vector3f rot_box;   
    if (drill_l[1]<=0)
    {   
        rot_box<<0.0,0.0,-acos(cosRtoD);   
    }
    else
    {
        rot_box<<0.0,0.0,acos(cosRtoD);   
    }
    
    box.setRotation(rot_box);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filter(new pcl::PointCloud<pcl::PointXYZ>);
    box.filter(*cloud_in_filter);
    pcl::copyPointCloud(*cloud_in_, *cloud_in);
    if (pcl::io::loadPLYFile("/home/yuhan/evl/drc_drill_2.ply", *cloud_model)<0){
        PCL_ERROR("drc_drill_2.ply is not found");
    }
    
    *cloud_model_in = *cloud_model;


    /** Use "Moving Least Squares" method for smoothing
        The input of MLS must be type PointXYZ*/

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_in_XYZ->resize(cloud_in->points.size());

    for(int index = 0; index < cloud_in->points.size(); index++) {
        cloud_in_XYZ->points[index].x=cloud_in->points[index].x;
        cloud_in_XYZ->points[index].y=cloud_in->points[index].y;
        cloud_in_XYZ->points[index].z=cloud_in->points[index].z;
    }

    /** create kdtree */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_MLS (new pcl::search::KdTree<pcl::PointXYZ>);
    
    /** Output of MLS, must be type PointNormal */ 
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);

    pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointNormal> mls; 
    mls.setComputeNormals (true);
    mls.setInputCloud (cloud_in_XYZ);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree_MLS);
    mls.setSearchRadius (0.02); //0.02 
 
    /** Upsampling using RANDOM_UNIFORM_DENSITY */
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointNormal>::RANDOM_UNIFORM_DENSITY);
    mls.setPointDensity(50); //50  
    
    /** Upsampling using SAMPLE_LOCAL_PLANE */
    mls.process (*mls_points); 
    *cloud_in=*mls_points;//

    /** segment the plane in the poincloud */
    pcl::SACSegmentation<pcl::PointNormal> seg;
    pcl::ExtractIndices<pcl::PointNormal> extract;
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.0115);
    seg.setInputCloud(cloud_in);
    seg.segment(*inliers_plane, *coefficients_plane);

    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers_plane);
    extract.setNegative(true);
    extract.filter(*cloud_object);
    
    /** Euclidean Clustering */
    /** build kdtree */
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree_ec (new pcl::search::KdTree<pcl::PointNormal>);
    tree_ec->setInputCloud (cloud_object);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
    ec.setClusterTolerance(0.015);  /** 1.5cm */
    ec.setMinClusterSize(100);  
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree_ec);
    ec.setInputCloud(cloud_object);
    ec.extract(cluster_indices);

    /** Select cluster with most pointcloud */
    int maxSize=0;
    int currentSize=0;
    std::vector<pcl::PointIndices>::const_iterator coreCloudIt;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointNormal>);
	
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        /** keep update max_number of cluster into coreCloud */ 
        currentSize=it->indices.size();
	    if(currentSize>maxSize){
	       coreCloudIt=it;
	       maxSize=currentSize;
	    }
    }

    /** Dump pointclouds of coreCloud into cloud_cluster */
    for (std::vector<int>::const_iterator pit = coreCloudIt->indices.begin(); pit != coreCloudIt->indices.end(); pit++) {
        cloud_cluster->points.push_back (cloud_object->points[*pit]);
    }

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    *cloud_object=*cloud_cluster;//



    pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor(true);
    sor.setInputCloud(cloud_object);
    sor.setMeanK(20);
    sor.setStddevMulThresh(0.1);
    sor.filter(*cloud_in);
    cloud_object=cloud_in;
    
    transformation_matrix = Eigen::Matrix4d::Identity();
    transformation_matrix(1,1)= 0;   
    transformation_matrix(1,2)= -1;   
    transformation_matrix(2,1)= 1;   
    transformation_matrix(2,2)= 0;   
    
    
    pcl::transformPointCloud (*cloud_model, *cloud_mid, transformation_matrix);
    *cloud_model = *cloud_mid;

    

    pcl::PCA<pcl::PointNormal> pca;
    pca.setInputCloud(cloud_object);
    Eigen::Matrix3f eigenVector_object=pca.getEigenVectors();
    Eigen::Vector3f eigenValue_object=pca.getEigenValues();

    pcl::compute3DCentroid (*cloud_object, centroid_object);
    
    pcl::ModelCoefficients line_coeff_obj_ev1;
    line_coeff_obj_ev1.values.resize(6);
    line_coeff_obj_ev1.values[0]=centroid_object(0);
    line_coeff_obj_ev1.values[1]=centroid_object(1);
    line_coeff_obj_ev1.values[2]=centroid_object(2);
    line_coeff_obj_ev1.values[3]=eigenVector_object(0,0);
    line_coeff_obj_ev1.values[4]=eigenVector_object(1,0);
    line_coeff_obj_ev1.values[5]=eigenVector_object(2,0);
     
    pcl::ModelCoefficients line_coeff_obj_ev2;
    line_coeff_obj_ev2.values.resize(6);
    line_coeff_obj_ev2.values[0]=centroid_object(0);
    line_coeff_obj_ev2.values[1]=centroid_object(1);
    line_coeff_obj_ev2.values[2]=centroid_object(2);

    float ipVectorX=secondClick.x-firstClick.x;
    float ipVectorY=secondClick.y-firstClick.y;
    float ipVectorZ=secondClick.z-firstClick.z;
    float magnitude=sqrt(ipVectorX*ipVectorX+ipVectorY*ipVectorY+ipVectorZ*ipVectorZ);
    Eigen::Vector3f indicated_principle_vector (ipVectorX/magnitude,ipVectorY/magnitude,ipVectorZ/magnitude);/** normalization */

    float dotResult=0;
    float result;
    int n=0;
    int i;
    for (i=0;i<3;i++){
      Eigen::Vector3f pV (eigenVector_object(0,i),eigenVector_object(1,i),eigenVector_object(2,i));
      result=std::abs(pV.dot(indicated_principle_vector));/** absolute value */
      if(result>dotResult){
	     dotResult=result; 
         n=i;
      }
    }  

    Eigen::Vector3f object_axis_init (eigenVector_object(0,n),eigenVector_object(1,n),0);
    
    if(indicated_principle_vector.dot(object_axis_init)>=0){
      line_coeff_obj_ev2.values[3]=eigenVector_object(0,n);
      line_coeff_obj_ev2.values[4]=eigenVector_object(1,n);
      line_coeff_obj_ev2.values[5]=eigenVector_object(2,n);
    }else{
      line_coeff_obj_ev2.values[3]=-eigenVector_object(0,n);
      line_coeff_obj_ev2.values[4]=-eigenVector_object(1,n);
      line_coeff_obj_ev2.values[5]=-eigenVector_object(2,n);

    }



    pcl::ModelCoefficients line_coeff_obj_ev3;
    line_coeff_obj_ev3.values.resize(6);
    line_coeff_obj_ev3.values[0]=centroid_object(0);
    line_coeff_obj_ev3.values[1]=centroid_object(1);
    line_coeff_obj_ev3.values[2]=centroid_object(2);
    line_coeff_obj_ev3.values[3]=eigenVector_object(0,2);
    line_coeff_obj_ev3.values[4]=eigenVector_object(1,2);
    line_coeff_obj_ev3.values[5]=eigenVector_object(2,2);
    

    pca.setInputCloud(cloud_model);
    Eigen::Matrix3f eigenVector_model=pca.getEigenVectors();  
    Eigen::Vector3f eigenValue_model=pca.getEigenValues();
    pcl::compute3DCentroid (*cloud_model, centroid_model);
    
    pcl::ModelCoefficients line_coeff_mod_ev1;
    line_coeff_mod_ev1.values.resize(6);
    line_coeff_mod_ev1.values[0]=centroid_model(0);
    line_coeff_mod_ev1.values[1]=centroid_model(1);
    line_coeff_mod_ev1.values[2]=centroid_model(2);
    line_coeff_mod_ev1.values[3]=eigenVector_model(0,0);
    line_coeff_mod_ev1.values[4]=eigenVector_model(1,0);
    line_coeff_mod_ev1.values[5]=eigenVector_model(2,0);
    
    
    pcl::ModelCoefficients line_coeff_mod_ev2;
    line_coeff_mod_ev2.values.resize(6);
    line_coeff_mod_ev2.values[0]=centroid_model(0);
    line_coeff_mod_ev2.values[1]=centroid_model(1);
    line_coeff_mod_ev2.values[2]=centroid_model(2);
    line_coeff_mod_ev2.values[3]=-eigenVector_model(0,1);
    line_coeff_mod_ev2.values[4]=-eigenVector_model(1,1);
    line_coeff_mod_ev2.values[5]=-eigenVector_model(2,1);
    
    pcl::ModelCoefficients line_coeff_mod_ev3;
    line_coeff_mod_ev3.values.resize(6);
    line_coeff_mod_ev3.values[0]=centroid_model(0);
    line_coeff_mod_ev3.values[1]=centroid_model(1);
    line_coeff_mod_ev3.values[2]=centroid_model(2);
    line_coeff_mod_ev3.values[3]=eigenVector_model(0,2);
    line_coeff_mod_ev3.values[4]=eigenVector_model(1,2);
    line_coeff_mod_ev3.values[5]=eigenVector_model(2,2);






    /** project the first principle axis to x-y plane */
    Eigen::Vector3f model_axis (line_coeff_mod_ev2.values[3],line_coeff_mod_ev2.values[4],0);
    Eigen::Vector3f object_axis (line_coeff_obj_ev2.values[3],line_coeff_obj_ev2.values[4],0); 
    
    /** calculate the angle between model_axis and object_axis */
    float cosAngle = model_axis.dot(object_axis)/(sqrt(model_axis.dot(model_axis))*sqrt(object_axis.dot(object_axis)));
    float sinAngle = sin(acos(cosAngle));(eigenVector_object(0,1),eigenVector_object(1,1),0);
    
    /** determine the turning direction */
    Eigen::Vector3f test_axis (line_coeff_mod_ev3.values[3],line_coeff_mod_ev3.values[4],0);
    if (object_axis.dot(test_axis)>0){
      sinAngle=-sinAngle;
    }

    
    transformation_matrix_pca = Eigen::Matrix4d::Identity();

    transformation_matrix_pca(0,0)=cosAngle;
    transformation_matrix_pca(0,1)=-sinAngle;
    transformation_matrix_pca(1,0)=sinAngle;
    transformation_matrix_pca(1,1)=cosAngle;
    
    pcl::transformPointCloud (*cloud_model, *cloud_mid, transformation_matrix_pca);
    *cloud_model = *cloud_mid;
        
    transformation_matrix = transformation_matrix_pca*transformation_matrix;
    
    /** centroid alignment */
    pcl::compute3DCentroid (*cloud_model, centroid_model);
    pcl::compute3DCentroid (*cloud_object, centroid_object);
    distance_centroid = centroid_object - centroid_model;

    transformation_matrix_centroid = Eigen::Matrix4d::Identity();
    transformation_matrix_centroid(0,3) = distance_centroid(0);
    transformation_matrix_centroid(1,3) = distance_centroid(1);
    transformation_matrix_centroid(2,3) = distance_centroid(2); 

    pcl::transformPointCloud (*cloud_model, *cloud_mid, transformation_matrix_centroid);

    *cloud_model = *cloud_mid;
    transformation_matrix = transformation_matrix_centroid*transformation_matrix;
    
    /** ICP alignment */
    pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    
    ne.setInputCloud(cloud_model);
    ne.setRadiusSearch(0.01);
    ne.compute(*cloud_model_normal);
    
    pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> ne2;
    ne2.setInputCloud(cloud_object);
    ne2.setRadiusSearch(0.01);
    ne2.compute(*cloud_object_normal);
    

    int pointNumber = cloud_model->points.size();
    for(int index = 0; index < pointNumber; index++) {
        cloud_model_normal->points[index].x=cloud_model->points[index].x;
        cloud_model_normal->points[index].y=cloud_model->points[index].y;
        cloud_model_normal->points[index].z=cloud_model->points[index].z;
    }
    

    pointNumber = cloud_object->points.size();
    for(int index = 0; index < pointNumber; index++) {
        cloud_object_normal->points[index].x=cloud_object->points[index].x;
        cloud_object_normal->points[index].y=cloud_object->points[index].y;
        cloud_object_normal->points[index].z=cloud_object->points[index].z;
    }
    
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
    icp.setMaximumIterations(300);
    icp.setMaxCorrespondenceDistance(0.05);
    icp.setTransformationEpsilon(1e-8);
    icp.setInputSource (cloud_model_normal);
    icp.setInputTarget (cloud_object_normal);
    icp.align(*cloud_mid_normal);
    transformation_matrix_icp = icp.getFinalTransformation().cast<double>();
    icp.setMaximumIterations(1);


    transformation_matrix = transformation_matrix_icp*transformation_matrix;
    std::cout << "Final Transformation" <<std::endl;   
    std::cout << transformation_matrix <<std::endl;
    double score=icp.getFitnessScore(0.1);
    std::cout<< "fitnessScore: "<<score<<std::endl;
    
    Eigen::Matrix3f rot;
    
    rot(0,0) = transformation_matrix(0,0);
    rot(1,0) = transformation_matrix(1,0);
    rot(2,0) = transformation_matrix(2,0);

    rot(0,1) = transformation_matrix(0,1);
    rot(1,1) = transformation_matrix(1,1);
    rot(2,1) = transformation_matrix(2,1);
    
    rot(0,2) = transformation_matrix(0,2);
    rot(1,2) = transformation_matrix(1,2);
    rot(2,2) = transformation_matrix(2,2);
    
    Eigen::Quaternion<float> quat(rot); 

    this->model_coefficients_->at(0) = 3.0;
    this->model_coefficients_->at(1) = transformation_matrix(0,3);
    this->model_coefficients_->at(2) = transformation_matrix(1,3);
    this->model_coefficients_->at(3) = transformation_matrix(2,3);
    this->model_coefficients_->at(4) = quat.w();
    this->model_coefficients_->at(5) = quat.x();
    this->model_coefficients_->at(6) = quat.y();
    this->model_coefficients_->at(7) = quat.z();


    Eigen::Matrix3f rot_palm;
    
    Eigen::Vector3f body2drill(transformation_matrix(0,3)-robot_x,transformation_matrix(1,3)-robot_y,0);
    Eigen::Vector3f drill_x(transformation_matrix(0,0),transformation_matrix(1,0),0);
        

    if (body2drill.dot(drill_x) < 0) 
    {
        rot_palm(0,1) = -transformation_matrix(0,0);  /** y-axis of the palm is -x axis of the drill */
        rot_palm(1,1) = -transformation_matrix(1,0);  /** x-axis */
        rot_palm(2,1) = -transformation_matrix(2,0);


        rot_palm(0,2) = transformation_matrix(0,2);  /** z-axis of the palm is the -y axis of the drill */
        rot_palm(1,2) = transformation_matrix(1,2);  /** y axis is point to the tip */
        rot_palm(2,2) = transformation_matrix(2,2);
    }
    else
    {
        rot_palm(0,1) = transformation_matrix(0,0);
        rot_palm(1,1) = transformation_matrix(1,0);
        rot_palm(2,1) = transformation_matrix(2,0);
    
        rot_palm(0,2) = -transformation_matrix(0,2);
        rot_palm(1,2) = -transformation_matrix(1,2);
        rot_palm(2,2) = -transformation_matrix(2,2);
    }
    rot_palm(0,0) = transformation_matrix(0,1);  /** x-axis is of the palm is z axis of drill */
    rot_palm(1,0) = transformation_matrix(1,1);  /** z-axis is point up */
    rot_palm(2,0) = transformation_matrix(2,1);
    Eigen::Quaternion<float> quat_palm(rot_palm); 
    
    Eigen::Vector3f offset(0,-0.046,-0.04);
    Eigen::Vector3f translate_offset;
    translate_offset = rot*offset;

    this->gripper_coefficients_->at(0) = transformation_matrix(0,3)+translate_offset(0);
    this->gripper_coefficients_->at(1) = transformation_matrix(1,3)+translate_offset(1);
    this->gripper_coefficients_->at(2) = transformation_matrix(2,3)+translate_offset(2);
    this->gripper_coefficients_->at(3) = quat_palm.w();
    this->gripper_coefficients_->at(4) = quat_palm.x();
    this->gripper_coefficients_->at(5) = quat_palm.y();
    this->gripper_coefficients_->at(6) = quat_palm.z();


    pcl::transformPointCloud (*cloud_model_in, *cloud_mid_normal, transformation_matrix);
    pcl::copyPointCloud(*cloud_in_filter, this->cloud_out_->at(0));


    }
}
    

