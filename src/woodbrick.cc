/**
 * This is the installation of the woodbrick algorithm class 
 * @version 1.0.0
 * @author Yuhan Long <yuhanlon@andrew.cmu.edu>
 * All right reserved 
 */

#include "woodbrick.h"

namespace evl{
    GeometryAnalyzerWoodbrick::GeometryAnalyzerWoodbrick(
            const boost::shared_ptr< pcl::PointCloud <PointT> > cloud_in, 
            const boost::shared_ptr<std::vector<PointT> > click_points,
            const boost::shared_ptr<std::vector<double> > robot_pose)
    {
        /** constructor function, setting click points, input point cloud and robot pose*/
        ROS_ERROR("[woodbrick] Entering woodbrick algorithm initiation");
        click_points_ = click_points;
        cloud_in_  = cloud_in;
        robot_pose_  = robot_pose;

        /** reinstantiate the model coefficients and gripper pose */
        ROS_ERROR("[woodbrick] Preparing model coefficients reset");
        model_coefficients_.reset(new std::vector<double>);
        model_coefficients_->resize(11);
        gripper_coefficients_.reset(new std::vector<double>);
        gripper_coefficients_->resize(7);
        
        /** get two click point from click_point vector */
        ROS_ERROR("[woodbrick] Model coefficients reset");
        firstClick = click_points->at(0);
        secondClick = click_points->at(1);
        ROS_ERROR("[woodbrick] Click point set");
    }

    
    void GeometryAnalyzerWoodbrick::compute()
    {
    /** Read the point cloud from documnents */
    PointCloudT::Ptr cloud_in_raw (new PointCloudT);
    PointCloudT::Ptr cloud_in (new PointCloudT);

    cloud_in_raw = cloud_in_;
    

    
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
    ROS_ERROR("[woodbrick] robot_pose size is %f, %f, %f",robot_x,robot_y,robot_z);


    /** Filtering using Moving Least Squares method */
    /** Create a KD-Tree */
    pcl::search::KdTree<PointT>::Ptr tree_MLS (new pcl::search::KdTree<PointT>);

    /** Output has the PointNormal type in order to store the normals calculated by MLS */
    PointCloudT::Ptr mls_points (new PointCloudT);

    /** Init object */
    pcl::MovingLeastSquares<PointT, PointT> mls;

    mls.setComputeNormals (false);

    /** Set parameters */
    mls.setInputCloud (cloud_in_raw);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree_MLS);
    mls.setSearchRadius (0.03);
    
    /** Reconstruct */
    mls.process (*mls_points);
    
    cloud_in = mls_points;

    /** Extract the K nearest point around the clicked point */
    std::vector <float> kDistances;
    std::vector <int> kIndices;
    pcl::IndicesPtr kIndicesPtr (new std::vector<int> (kIndices));
    
    kDistances.resize(100);
    kIndicesPtr->resize(100);
    
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud_in);
    kdtree.nearestKSearch (firstClick, 100, *kIndicesPtr, kDistances);

    const PointCloudT cloud_in_const = *cloud_in;
    PointCloudT::Ptr nearestNeighbourPoints ( new PointCloudT(cloud_in_const,*kIndicesPtr));
    
    /** Do ransac with the 100 nearest points of the click point to get the first plane */
    pcl::SACSegmentation<PointT> segPlane;
    pcl::PointIndices::Ptr first_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_first_ptr(new pcl::ModelCoefficients);
    segPlane.setOptimizeCoefficients(true);
    segPlane.setModelType(pcl::SACMODEL_PLANE);
    /** change model to plane */
    segPlane.setMethodType(pcl::SAC_RANSAC);
    segPlane.setMaxIterations(100); /** Number of iterations */

    segPlane.setDistanceThreshold(0.015);

    segPlane.setInputCloud(nearestNeighbourPoints);

    segPlane.segment(*first_plane, *coefficients_first_ptr);

    Eigen::VectorXf coefficients_first;
    coefficients_first.resize(4);
    coefficients_first[0] = coefficients_first_ptr->values[0];
    coefficients_first[1] = coefficients_first_ptr->values[1];
    coefficients_first[2] = coefficients_first_ptr->values[2];
    coefficients_first[3] = coefficients_first_ptr->values[3];

    /** aFlip normal vector if dot(origin2firstclick, firstplanenormal) > 0 to have normal towards origin */
    if (coefficients_first[0]*(firstClick.x-robot_x) + coefficients_first[1]*(firstClick.y-robot_y) + coefficients_first[2]*(firstClick.z-robot_z) > 0) {
        coefficients_first[0] = -coefficients_first[0];
        coefficients_first[1] = -coefficients_first[1];
        coefficients_first[2] = -coefficients_first[2];
        coefficients_first[3] = -coefficients_first[3];
    }
   
    /** Get all inliers in the point cloud */
    pcl::SampleConsensusModelPlane<PointT>::Ptr segPlaneSeg (new pcl::SampleConsensusModelPlane<PointT>(cloud_in));
    segPlaneSeg->setInputCloud(cloud_in);
    std::vector <int> firstIndices;
    pcl::IndicesPtr firstIndicesPtr (new std::vector<int> (firstIndices));
    segPlaneSeg->selectWithinDistance(coefficients_first,0.015,*firstIndicesPtr);
    PointCloudT::Ptr firstPlane (new PointCloudT(*cloud_in, *firstIndicesPtr)); /** Point cloud pointer to all first plane inliers */

    pcl::ModelCoefficients::Ptr first_plane_visual (new pcl::ModelCoefficients);
    first_plane_visual->values.resize(4);
    first_plane_visual->values[0]=coefficients_first[0];
    first_plane_visual->values[1]=coefficients_first[1];
    first_plane_visual->values[2]=coefficients_first[2];
    first_plane_visual->values[3]=coefficients_first[3];

    /** Estimate normals of first plane */
    /** Create the normal estimation class, and pass the input dataset to it */
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (firstPlane);

    /** Create an empty kdtree representation, and pass it to the normal estimation object. */
    /** Its content will be filled inside the object, based on the given input dataset (as no other search surface is given). */
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod (tree);
    ne.setViewPoint (robot_x,robot_y,robot_z);

    /** Output datasets */
    pcl::PointCloud<pcl::Normal>::Ptr firstnormals (new pcl::PointCloud<pcl::Normal>);

    ne.setKSearch(10);

    /** Compute the features */
    ne.compute (*firstnormals);

    /** Filter normals based on directions */
    std::vector<int> firstfilterind;
    float thres = 0.3;
    for (int i = 0; i < firstnormals->size(); i++) {
        if (firstnormals->at(i).normal_x > 1*coefficients_first[0] - thres &&
            firstnormals->at(i).normal_x < 1*coefficients_first[0] + thres &&
            firstnormals->at(i).normal_y > 1*coefficients_first[1] - thres &&
            firstnormals->at(i).normal_y < 1*coefficients_first[1] + thres &&
            firstnormals->at(i).normal_z > 1*coefficients_first[2] - thres &&
            firstnormals->at(i).normal_z < 1*coefficients_first[2] + thres)
        firstfilterind.push_back(i);
    }
  
    PointCloudT::Ptr firstFilter (new PointCloudT(*firstPlane, firstfilterind)); /** Select only points that pass the test */
    PointCloudT::Ptr firstFilterSOR (new PointCloudT); /** Container for after SOR */
  
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (firstFilter);
    sor.setMeanK (30);
    sor.setStddevMulThresh (0.4); /** Small is better for filtering, but too small will filter the good ones */
    sor.filter (*firstFilterSOR);

    /** Print out normal for first plane */
    /** Put variable here to avoid scope issue */
    pcl::PointCloud<pcl::Normal>::Ptr firstnormals2 (new pcl::PointCloud<pcl::Normal>);

    /** Create the normal estimation class, and pass the input dataset to it */
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne2;
    ne2.setInputCloud (firstFilterSOR);

    /** Create an empty kdtree representation, and pass it to the normal estimation object. */
    /** Its content will be filled inside the object, based on the given input dataset (as no other search surface is given). */
    pcl::search::KdTree<PointT>::Ptr tree2 (new pcl::search::KdTree<PointT> ());
    ne2.setSearchMethod (tree2);

    /** Use all neighbors in a sphere of radius 3cm */
    ne2.setKSearch(10);

    /** Compute the features */
    ne2.compute (*firstnormals2);

    /* Perform Euclidean clustering to get rid of floating point cloud chunks */
    PointCloudT::Ptr firstFilterSOREC (new PointCloudT); /** Container for after SOR and EC */

    pcl::search::KdTree<PointT>::Ptr treeec (new pcl::search::KdTree<PointT>);
    tree -> setInputCloud(firstFilterSOR);
    tree -> nearestKSearch (firstClick, 1, *kIndicesPtr, kDistances);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.04); /** Tolerance in meter(s)... can't be too smaller than pc resolution */
    ec.setMinClusterSize (100);
    ec.setSearchMethod(tree);
    ec.setInputCloud(firstFilterSOR);
    ec.extract(cluster_indices);
    

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
    bool is_clicked = false;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            if ((*kIndicesPtr)[0]==*pit) is_clicked = true;
            cloud_cluster->points.push_back (firstFilterSOR->points[*pit]);
        }

        cloud_cluster->width = firstFilterSOR->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        if (is_clicked) { 
            firstFilterSOREC = cloud_cluster;
            break;
        }
    }

    /* Measure the second plane */
    kdtree.nearestKSearch (secondClick, 100, *kIndicesPtr, kDistances);
    
    /** This point cloud is just for visualization
    PointCloudT::Ptr nearestNeighbourPointsSecond (new PointCloudT(cloud_in_const, *kIndicesPtr));
      
    /* Ransac around k nearest points to get the second plane */
    pcl::SACSegmentation<PointT> segPlane2;
    pcl::PointIndices::Ptr second_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_second_ptr(new pcl::ModelCoefficients);
    segPlane2.setOptimizeCoefficients(true);
    segPlane2.setModelType(pcl::SACMODEL_PARALLEL_PLANE); /** Change model to parallel plane */
    segPlane2.setMethodType(pcl::SAC_RANSAC);
    segPlane2.setMaxIterations(100); /** Number of iterations */
    segPlane2.setDistanceThreshold(0.015);
    Eigen::Vector3f axis (coefficients_first[0],coefficients_first[1],coefficients_first[2]);
    segPlane2.setAxis(axis); /** Set axis for the plane to be parallel to */
    segPlane2.setInputCloud(nearestNeighbourPointsSecond);

    segPlane2.segment(*second_plane, *coefficients_second_ptr);

    Eigen::VectorXf coefficients_second;
    coefficients_second.resize(4);
    coefficients_second[0] = coefficients_second_ptr->values[0];
    coefficients_second[1] = coefficients_second_ptr->values[1];
    coefficients_second[2] = coefficients_second_ptr->values[2];
    coefficients_second[3] = coefficients_second_ptr->values[3];

    /** Flip normal vector if dot(origin2secondclick, secondplanenormal) > 0 to have normal towards origin */
    if (coefficients_second[0]*(secondClick.x-robot_x) + coefficients_second[1]*(secondClick.y-robot_y) + coefficients_second[2]*(secondClick.z-robot_z) > 0) {
        coefficients_second[0] = -coefficients_second[0];
        coefficients_second[1] = -coefficients_second[1];
        coefficients_second[2] = -coefficients_second[2];
        coefficients_second[3] = -coefficients_second[3];
    }


    /** Get inliers on the second plane */
    pcl::SampleConsensusModelPlane<PointT>::Ptr segPlaneSeg2 (new pcl::SampleConsensusModelPlane<PointT>(cloud_in));
    segPlaneSeg2->setInputCloud(cloud_in);
    std::vector <int> secondIndices;
    pcl::IndicesPtr secondIndicesPtr (new std::vector<int> (secondIndices));
    segPlaneSeg2->selectWithinDistance(coefficients_second,0.015,*secondIndicesPtr);
    PointCloudT::Ptr secondPlane (new PointCloudT(*cloud_in, *secondIndicesPtr)); /** Point cloud pointer to all second plane inliers */

    pcl::ModelCoefficients::Ptr second_plane_visual (new pcl::ModelCoefficients);
    second_plane_visual->values.resize(4);
    second_plane_visual->values[0]=coefficients_second[0];
    second_plane_visual->values[1]=coefficients_second[1];
    second_plane_visual->values[2]=coefficients_second[2];
    second_plane_visual->values[3]=coefficients_second[3];

    /** Estimate normals of second plane */
    /** Create the normal estimation class, and pass the input dataset to it */
    ne.setInputCloud (secondPlane);

    /** Create an empty kdtree representation, and pass it to the normal estimation object. */
    /** Its content will be filled inside the object, based on the given input dataset (as no other search surface is given). */
    pcl::search::KdTree<PointT>::Ptr secondtree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod (secondtree);

    /** Output datasets */
    pcl::PointCloud<pcl::Normal>::Ptr secondnormals (new pcl::PointCloud<pcl::Normal>);

    ne.setKSearch(10);

    /** Compute the features */
    ne.compute (*secondnormals);

    /** Filter normals based on directions */
    std::vector<int> secondfilterind;
    float secondthres = 0.3;
    for (int i = 0; i < secondnormals->size(); i++) {
      if (secondnormals->at(i).normal_x > 1*coefficients_second[0] - secondthres &&
          secondnormals->at(i).normal_x < 1*coefficients_second[0] + secondthres &&
          secondnormals->at(i).normal_y > 1*coefficients_second[1] - secondthres &&
          secondnormals->at(i).normal_y < 1*coefficients_second[1] + secondthres &&
          secondnormals->at(i).normal_z > 1*coefficients_second[2] - secondthres &&
          secondnormals->at(i).normal_z < 1*coefficients_second[2] + secondthres)
        secondfilterind.push_back(i);
    }

    /** Do a stat outlier removal to get rid of (small) floating points */
    PointCloudT::Ptr secondFilter (new PointCloudT(*secondPlane, secondfilterind)); /** Select only points that pass the test */
    PointCloudT::Ptr secondFilterSOR (new PointCloudT); /** Container for after SOR */
  
    sor.setInputCloud (secondFilter);
    sor.setMeanK (30);
    sor.setStddevMulThresh (0.4); /** Small is better for filtering, but too small will filter the good ones */
    sor.filter (*secondFilterSOR);

    /* Print out normal for second plane */
    /** Put variable here to avoid scope issue */
    pcl::PointCloud<pcl::Normal>::Ptr secondnormals2 (new pcl::PointCloud<pcl::Normal>);
    /** Create the normal estimation class, and pass the input dataset to it */
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> secondne2;
    secondne2.setInputCloud (secondFilterSOR);

    /** Create an empty kdtree representation, and pass it to the normal estimation object. */
    /** Its content will be filled inside the object, based on the given input dataset (as no other search surface is given). */
    pcl::search::KdTree<PointT>::Ptr secondtree2 (new pcl::search::KdTree<PointT> ());
    secondne2.setSearchMethod (secondtree2);

    /** Use all neighbors in a sphere of radius 3cm */
    secondne2.setKSearch(10);

    /** Compute the features */
    secondne2.compute (*secondnormals2);

    /** Perform Euclidean clustering to get rid of floating point cloud chunks */
    PointCloudT::Ptr secondFilterSOREC (new PointCloudT); /** Container for after SOR and EC */

    std::vector<pcl::PointIndices> cluster_indices2;
    treeec -> setInputCloud(secondFilterSOR);
    treeec -> nearestKSearch (secondClick, 1, *kIndicesPtr, kDistances);
    ec.setClusterTolerance(0.04); /** Tolerance in meter(s)... can't be too smaller than pc resolution */
    ec.setMinClusterSize (100);
    ec.setSearchMethod(tree);
    ec.setInputCloud(secondFilterSOR);
    ec.extract(cluster_indices2);

    is_clicked = false;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices2.begin(); it != cluster_indices2.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            if ((*kIndicesPtr)[0]==*pit) is_clicked = true;
            cloud_cluster->points.push_back (secondFilterSOR->points[*pit]);
        }

        cloud_cluster->width = secondFilterSOR->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        if (is_clicked) { 
            secondFilterSOREC = cloud_cluster;
            break;
        }
    }

    /* Calculate the intersaction line */
    /* Ref. http:/**geomalgorithms.com/a05-_intersect-1.html */
    Eigen::Vector3f p1;
    p1[0] = first_plane_visual->values[0];
    p1[1] = first_plane_visual->values[1];
    p1[2] = first_plane_visual->values[2];

    float w1 = first_plane_visual->values[3];
    float a1 = first_plane_visual->values[0];
    float b1 = first_plane_visual->values[1];
    float c1 = first_plane_visual->values[2];

    Eigen::Vector3f p2;
    p2[0] = second_plane_visual->values[0];
    p2[1] = second_plane_visual->values[1];
    p2[2] = second_plane_visual->values[2];
    float w2 = second_plane_visual->values[3];
    float a2 = second_plane_visual->values[0];
    float b2 = second_plane_visual->values[1];
    float c2 = second_plane_visual->values[2];
    
    Eigen::Vector3f line;
    line = p1.cross(p2);
    float absX = (line[0]>=0 ? line[0] : -line[0]);
    float absY = (line[1]>=0 ? -line[1] : line[1]);
    float absZ = (line[2]>=0 ? -line[2] : line[2]);
    float px;
    float py;
    float pz;

    int maxc;
    if (absX >= absY) {
        if (absX >= absZ)
            maxc = 1;
        else
            maxc = 3;
    }
    else {
        if (absY >= absZ)
            maxc = 2;
        else
            maxc = 3;
    }

    switch(maxc) {
        case 1: 
            px = 0;
            pz = -(w1*b2-w2*b1)/(c1*b2-c2*b1);
            py = -(w1*c2-w2*c1)/(b1*c2-b2*c1);
            break;
        case 2:
            py = 0;
            pz = -(w1*a2-w2*a1)/(c1*a2-c2*a1);
            px = -(w1*c2-w2*c1)/(a1*c2-a2*c1);
            break;
        case 3:
            pz = 0;
            py = -(w1*a2-w2*a1)/(b1*a2-b2*a1);
            px = -(w1*b2-w2*b1)/(a1*c2-a2*b1);
    }
    
    pcl::ModelCoefficients::Ptr intersection_line (new pcl::ModelCoefficients);
    intersection_line->values.resize(6);
    intersection_line->values[0] = px;
    intersection_line->values[1] = py;
    intersection_line->values[2] = pz;

    intersection_line->values[3] = line[0];
    intersection_line->values[4] = line[1];
    intersection_line->values[5] = line[2];
    
    /* Project points on the line  */
    PointCloudT::Ptr addedpc(new PointCloudT((*firstFilterSOREC)+(*secondFilterSOREC)));

    pcl::ProjectInliers<PointT> proj_line; 
    proj_line.setModelType(pcl::SACMODEL_LINE);
    proj_line.setInputCloud(addedpc);
    proj_line.setModelCoefficients(intersection_line);
    PointCloudT::Ptr cloud_projected (new PointCloudT);
    proj_line.filter(*cloud_projected); /** Entire input point cloud projected onto this line... */


    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    /**float distance; */

    pcl::getMinMax3D(*cloud_projected, min_pt,max_pt);
    float distance1, distance2, distance3;
    distance1  = sqrt((max_pt[0]-min_pt[0])*(max_pt[0]-min_pt[0])+
                (max_pt[1]-min_pt[1])*(max_pt[1]-min_pt[1])+
                (max_pt[2]-min_pt[2])*(max_pt[2]-min_pt[2]));


    PointT line1Mid;
    line1Mid.x = (max_pt[0]+min_pt[0])/2;
    line1Mid.y = (max_pt[1]+min_pt[1])/2;
    line1Mid.z = (max_pt[2]+min_pt[2])/2;
    
    
    PointT vertex1;
    PointT vertex2;
    float vector_length = sqrt(line[0]*line[0]+line[1]*line[1]+line[2]*line[2]);
    vertex1.x = 0.5 * distance1 * line[0] / vector_length + line1Mid.x;
    vertex1.y = 0.5 * distance1 * line[1] / vector_length + line1Mid.y;
    vertex1.z = 0.5 * distance1 * line[2] / vector_length + line1Mid.z;

    vertex2.x = -0.5 * distance1 * line[0] / vector_length + line1Mid.x;
    vertex2.y = -0.5 * distance1 * line[1] / vector_length + line1Mid.y;
    vertex2.z = -0.5 * distance1 * line[2] / vector_length + line1Mid.z;

    /* Calculate the length of the other two edges  */
    pcl::ModelCoefficients::Ptr segPlane3 (new pcl::ModelCoefficients);
    segPlane3->values.resize(4);
    segPlane3->values[0]= line[0];
    segPlane3->values[1]= line[1];
    segPlane3->values[2]= line[2];
    segPlane3->values[3] = -line[0]*vertex1.x-line[1]*vertex1.y-line[2]*vertex1.z;
    
    /* Project the first plane on the third plane */
    PointCloudT::Ptr plane1_projected (new PointCloudT);
    PointCloudT::Ptr edge2_projected (new PointCloudT);

    proj_line.setModelType(pcl::SACMODEL_PLANE);
    proj_line.setInputCloud(firstFilterSOREC);
    proj_line.setModelCoefficients(first_plane_visual);
    proj_line.filter(*plane1_projected);
    
    float normalx = line[0]/sqrt(line[0]*line[0]+line[1]*line[1]+line[2]*line[2]);
    float normaly = line[1]/sqrt(line[0]*line[0]+line[1]*line[1]+line[2]*line[2]);
    float normalz = line[2]/sqrt(line[0]*line[0]+line[1]*line[1]+line[2]*line[2]);
    for(size_t i=0; i < plane1_projected->points.size(); ++i) {
        float scale = ((plane1_projected->points[i].x-vertex1.x)*normalx+
        (plane1_projected->points[i].y-vertex1.y)*normaly+
        (plane1_projected->points[i].z-vertex1.z)*normalz);
        edge2_projected->points.push_back (PointT(plane1_projected->points[i].x-scale*normalx,plane1_projected->points[i].y-scale*normaly,plane1_projected->points[i].z-scale*normalz));
    }
    edge2_projected->width = edge2_projected->points.size();
    edge2_projected->height = 1;
    edge2_projected->is_dense = true;

    /* Calculate the length of the other edge of the first plane */
    pcl::getMinMax3D(*edge2_projected, min_pt,max_pt);
    distance2  = sqrt((max_pt[0]-min_pt[0])*(max_pt[0]-min_pt[0])+
                (max_pt[1]-min_pt[1])*(max_pt[1]-min_pt[1])+
                (max_pt[2]-min_pt[2])*(max_pt[2]-min_pt[2]));
    
    PointT line2Mid;
    line2Mid.x = (max_pt[0]+min_pt[0])/2;
    line2Mid.y = (max_pt[1]+min_pt[1])/2;
    line2Mid.z = (max_pt[2]+min_pt[2])/2;
    
    /* Project the second plane on the third plane */
    PointCloudT::Ptr plane2_projected (new PointCloudT);
    PointCloudT::Ptr edge3_projected (new PointCloudT);
    
    proj_line.setModelType(pcl::SACMODEL_PLANE);
    proj_line.setInputCloud(secondFilterSOREC);
    proj_line.setModelCoefficients(second_plane_visual);
    proj_line.filter(*plane2_projected);

    for(size_t i=0; i< plane2_projected -> points.size();++i) {
        float scale = ((plane2_projected->points[i].x-vertex1.x)*normalx+
        (plane2_projected->points[i].y-vertex1.y)*normaly+
        (plane2_projected->points[i].z-vertex1.z)*normalz);
        edge3_projected->points.push_back (PointT(plane2_projected->points[i].x-scale*normalx,plane2_projected->points[i].y-scale*normaly,plane2_projected->points[i].z-scale*normalz));
    }
    edge3_projected->width = edge3_projected->points.size ();
    edge3_projected->height = 1;
    edge3_projected->is_dense = true;

    /* Calculate the length of the other edge of the second plane */
    pcl::getMinMax3D(*edge3_projected, min_pt,max_pt);
    distance3  = sqrt((max_pt[0]-min_pt[0])*(max_pt[0]-min_pt[0])+
                (max_pt[1]-min_pt[1])*(max_pt[1]-min_pt[1])+
                (max_pt[2]-min_pt[2])*(max_pt[2]-min_pt[2]));
   
    PointT line3Mid;
    line3Mid.x = (max_pt[0]+min_pt[0])/2;
    line3Mid.y = (max_pt[1]+min_pt[1])/2;
    line3Mid.z = (max_pt[2]+min_pt[2])/2;

    /* Calculate the other six points  */
    PointT vertex3;
    PointT vertex4;
    PointT vertex5;
    PointT vertex6;
    PointT vertex7;
    PointT vertex8;
    
    vertex3.x = 2*(line2Mid.x-vertex1.x)+vertex1.x;
    vertex3.y = 2*(line2Mid.y-vertex1.y)+vertex1.y;
    vertex3.z = 2*(line2Mid.z-vertex1.z)+vertex1.z;

    vertex4.x = 2*(line2Mid.x-vertex1.x)+vertex2.x;
    vertex4.y = 2*(line2Mid.y-vertex1.y)+vertex2.y;
    vertex4.z = 2*(line2Mid.z-vertex1.z)+vertex2.z;

    vertex5.x = vertex1.x-distance3*coefficients_first[0];
    vertex5.y = vertex1.y-distance3*coefficients_first[1];
    vertex5.z = vertex1.z-distance3*coefficients_first[2];
   
    vertex6.x = vertex1.x-distance3*coefficients_first[0];
    vertex6.y = vertex1.y-distance3*coefficients_first[1];
    vertex6.z = vertex1.z-distance3*coefficients_first[2];
    
    vertex7.x = vertex1.x-distance3*coefficients_first[0];
    vertex7.y = vertex1.y-distance3*coefficients_first[1];
    vertex7.z = vertex1.z-distance3*coefficients_first[2];
    
    vertex8.x = vertex1.x-distance3*coefficients_first[0];
    vertex8.y = vertex1.y-distance3*coefficients_first[1];
    vertex8.z = vertex1.z-distance3*coefficients_first[2];



    /* Printing results */

    std::cout << "Filtering normal for first plane... ";
    std::cout << "\nA total of: " << firstFilterSOR->size() << '\n';
    std::cout << "Clustering first plane... ";
    std::cout << "\nA total of: " << firstFilterSOREC->size() << '\n';

    std::cout << "Filtering normal for second plane... ";
    std::cout << "\nA total of: " << secondFilterSOR->size() << '\n';
    std::cout << "Clustering second plane... ";
    std::cout << "\nA total of: " << secondFilterSOREC->size() << '\n';
    
    
    std::vector<double> distance(3);
    distance[0] = distance1;
    distance[1] = distance2;
    distance[2] = distance3;

    std::vector<PointT> vectorPoint(3);
    vectorPoint[0] = vertex2;
    vectorPoint[1] = vertex3;
    vectorPoint[2] = vertex5;
    
    std::vector<int> distanceIdx(3);

    
    distanceIdx[0] = 0; 
    distanceIdx[1] = 1;  
    distanceIdx[2] = 2;
    std::cout << "====================distance of the first edge=================" << std::endl;
    std::cout << distance[distanceIdx[0]] << std::endl;
    std::cout << "====================distance of the second edge=================" << std::endl;
    std::cout << distance[distanceIdx[1]] << std::endl;
    std::cout << "====================distance of the third edge=================" << std::endl;
    std::cout << distance[distanceIdx[2]] << std::endl;

    Eigen::Vector3f vectorX;
    Eigen::Vector3f vectorY;
    Eigen::Vector3f vectorZ;
    double length, width, height;
    length = distance[distanceIdx[0]];
    width = distance[distanceIdx[1]];
    height = distance[distanceIdx[2]];
    
    PointT vertexX = vectorPoint[distanceIdx[0]];
    PointT vertexZ = vectorPoint[distanceIdx[2]];

    vectorX[0] = vertexX.x - vertex1.x;
    vectorX[1] = vertexX.y - vertex1.y;
    vectorX[2] = vertexX.z - vertex1.z;
    
    vectorZ[0] = -first_plane_visual->values[0];
    vectorZ[1] = -first_plane_visual->values[1];
    vectorZ[2] = -first_plane_visual->values[2];



    vectorX = vectorX.normalized();
    vectorZ = vectorZ.normalized();

    vectorY = vectorZ.cross(vectorX);

    Eigen::Matrix3f rot;
    rot(0,0) = vectorX[0];
    rot(1,0) = vectorX[1];
    rot(2,0) = vectorX[2];

    rot(0,1) = vectorY[0];
    rot(1,1) = vectorY[1];
    rot(2,1) = vectorY[2];

    rot(0,2) = vectorZ[0];
    rot(1,2) = vectorZ[1];
    rot(2,2) = vectorZ[2];

    Eigen::Quaternion<float> quat(rot); 
    
    std::cout<<"quat:"<<quat.x() <<" "<< quat.y() <<" "<< quat.z() << " "<< quat.w()<<std::endl;
    this->model_coefficients_->at(0) = 2.0;            
    this->model_coefficients_->at(1) = line1Mid.x+line2Mid.x+line3Mid.x-2*vertex1.x;            
    this->model_coefficients_->at(2) = line1Mid.y+line2Mid.y+line3Mid.y-2*vertex1.y;            
    this->model_coefficients_->at(3) = line1Mid.z+line2Mid.z+line3Mid.z-2*vertex1.z;
    this->model_coefficients_->at(4) = quat.w();            
    this->model_coefficients_->at(5) = quat.x();            
    this->model_coefficients_->at(6) = quat.y();            
    this->model_coefficients_->at(7) = quat.z();            
    this->model_coefficients_->at(8) = length;            
    this->model_coefficients_->at(9) = width;            
    this->model_coefficients_->at(10) = height; 

    Eigen::Matrix3f rot_palm;
    
    Eigen::Vector3f face1Mid;
    face1Mid[0] = vectorX[0];
    face1Mid[1] = vectorX[1];
    face1Mid[2] = vectorX[2];

    
    Eigen::Vector3f mid2Click;
    mid2Click << -(line1Mid.x+line2Mid.x+line3Mid.x-2*vertex1.x-firstClick.x),-(line1Mid.y+line2Mid.y+line3Mid.y-2*vertex1.y-firstClick.y),-(line1Mid.z+line2Mid.z+line3Mid.z-2*vertex1.z-firstClick.z);
    
    Eigen::Vector3f midClickPoint;

    midClickPoint = ((mid2Click.dot(face1Mid))/(face1Mid.dot(face1Mid)))*face1Mid;

    
    


    rot_palm(0,0) = vectorY[0];
    rot_palm(1,0) = vectorY[1];
    rot_palm(2,0) = vectorY[2];    
    
    rot_palm(0,1) = vectorZ[0];
    rot_palm(1,1) = vectorZ[1];
    rot_palm(2,1) = vectorZ[2];

    rot_palm(0,2) = vectorX[0];
    rot_palm(1,2) = vectorX[1];
    rot_palm(2,2) = vectorX[2];
    

    Eigen::Quaternion<float> quat_palm(rot_palm);
    this->gripper_coefficients_->at(0) = midClickPoint[0]+line1Mid.x+line2Mid.x+line3Mid.x-2*vertex1.x;
    this->gripper_coefficients_->at(1) = midClickPoint[1]+line1Mid.y+line2Mid.y+line3Mid.y-2*vertex1.y;
    this->gripper_coefficients_->at(2) = midClickPoint[2]+line1Mid.z+line2Mid.z+line3Mid.z-2*vertex1.z;
    this->gripper_coefficients_->at(3) = quat_palm.w();
    this->gripper_coefficients_->at(4) = quat_palm.x();
    this->gripper_coefficients_->at(5) = quat_palm.y();
    this->gripper_coefficients_->at(6) = quat_palm.z();


    }
}
