/**
 * This is the implememtation of one click point woodbrick algorithm class 
 * @version 1.0.0
 * @author Yuhan Long <yuhanlon@andrew.cmu.edu>
 * All right reserved 
 */

#include "woodbrick1.h"

namespace evl{
    GeometryAnalyzerWoodbrick1::GeometryAnalyzerWoodbrick1(
            const boost::shared_ptr< pcl::PointCloud <PointT> > cloud_in, 
            const boost::shared_ptr<std::vector<PointT> > click_points)
    {
        /** constructor function, setting click points and input point cloud */
        ROS_INFO("[woodbrick1] Entering woodbrick algorithm initiation");
        click_points_ = click_points;
        cloud_in_  = cloud_in;

        /** reinstantiate the model coefficients and griper pose */ 
        ROS_INFO("[woodbrick1] Preparing model coefficients reset");
        model_coefficients_.reset(new std::vector<double>);
        model_coefficients_->resize(11);
        gripper_coefficients_.reset(new std::vector<double>);
        gripper_coefficients_->resize(7);

        /** get the click point */
        ROS_INFO("[woodbrick1] Model coefficients reset");
        firstClick = click_points->at(0);
        ROS_INFO("[woodbrick1] Click point set");
    }

    /** Calculates the angle between the two unit vectors. */
    float GeometryAnalyzerWoodbrick1::unitVectorAngle(float x1, float y1, float z1, float x2, float y2, float z2) {
        return acos(x1*x2 + y1*y2 + z1*z2);
    }

    /** Calculates the Euclidean distance between the two points. */
    float GeometryAnalyzerWoodbrick1::distL2(float x1, float y1, float z1, float x2, float y2, float z2) {
        return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
    }

    void GeometryAnalyzerWoodbrick1::compute()
    {
     /** Parse commands */
    float extrude_in = 2.0;

    /** Read the point cloud from documnents */   
    PointCloudT::Ptr cloud_in_raw(new PointCloudT);
    PointCloudT::Ptr cloud_in(new PointCloudT);
    cloud_in_raw = cloud_in_;
    
    /** Filtering using Moving Least Squares method */

    /** Create a KD-Tree */
    pcl::search::KdTree<PointT>::Ptr tree_MLS(new pcl::search::KdTree<PointT>);

    /** Output has the PointNormal type in order to store the normals calculated by MLS */
    PointCloudT::Ptr mls_points(new PointCloudT);

    /** Init object */
    pcl::MovingLeastSquares<PointT, PointT> mls;

    mls.setComputeNormals(false);

    /** Set parameters */
    mls.setInputCloud(cloud_in_raw);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree_MLS);
    mls.setSearchRadius(0.03);
    
    /** Reconstruct */
    mls.process(*mls_points);
    
    cloud_in = mls_points; /** Use this line to use MLS */


    /**
     *FIRST PLANE
     *Extract the K nearest point around the clicked point
     */

    std::vector <float> kDistances;
    std::vector <int> kIndices;
    pcl::IndicesPtr kIndicesPtr(new std::vector<int>(kIndices));
    
    kDistances.resize(100);
    kIndicesPtr->resize(100);
    
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud_in);
    kdtree.nearestKSearch(firstClick, 100, *kIndicesPtr, kDistances);

    const PointCloudT cloud_in_const = *cloud_in;
    PointCloudT::Ptr nearestNeighbourPoints( new PointCloudT(cloud_in_const, *kIndicesPtr));
    
    /**
     *Do ransac with the k nearest points of the click point to get the first plane
     */   
    
    pcl::SACSegmentation<PointT> segPlane;
    pcl::ExtractIndices<PointT> extract;
    pcl::PointIndices::Ptr first_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_first_ptr(new pcl::ModelCoefficients);
    segPlane.setOptimizeCoefficients(true);
    segPlane.setModelType(pcl::SACMODEL_PLANE);/** change model to plane */
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

    if (coefficients_first[0]*firstClick.x + coefficients_first[1]*firstClick.y + coefficients_first[2]*firstClick.z > 0) {
        coefficients_first[0] = -coefficients_first[0];
        coefficients_first[1] = -coefficients_first[1];
        coefficients_first[2] = -coefficients_first[2];
        coefficients_first[3] = -coefficients_first[3];
    }
   
    /**
     *Get all inliers in the point cloud
     */   
    pcl::SampleConsensusModelPlane<PointT>::Ptr segPlaneSeg(new pcl::SampleConsensusModelPlane<PointT>(cloud_in));
    segPlaneSeg->setInputCloud(cloud_in);
    std::vector <int> firstIndices;
    pcl::IndicesPtr firstIndicesPtr(new std::vector<int>(firstIndices));
    segPlaneSeg->selectWithinDistance(coefficients_first,0.015,*firstIndicesPtr);
    PointCloudT::Ptr firstPlane(new PointCloudT(cloud_in_const, *firstIndicesPtr)); /** Point cloud pointer to all first plane inliers */

    pcl::ModelCoefficients::Ptr first_plane_visual(new pcl::ModelCoefficients);
    first_plane_visual->values.resize(4);
    first_plane_visual->values[0]=coefficients_first[0];
    first_plane_visual->values[1]=coefficients_first[1];
    first_plane_visual->values[2]=coefficients_first[2];
    first_plane_visual->values[3]=coefficients_first[3];

    /**
     *Estimate normals of first plane
     */   

    /** Create the normal estimation class, and pass the input dataset to it */
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(firstPlane);

    /** Create an empty kdtree representation, and pass it to the normal estimation object. */
    /** Its content will be filled inside the object, based on the given input dataset(as no other search surface is given). */
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);

    /** Output datasets */
    pcl::PointCloud<pcl::Normal>::Ptr firstnormals(new pcl::PointCloud<pcl::Normal>);

    ne.setKSearch(30);

    /** Compute the features */
    ne.compute(*firstnormals);

    /** Filter normals based on directions */
    std::vector<int> firstfilterind;
    float firstthres = 25.0 * M_PI / 180.0;
    for (int i = 0; i < firstnormals->size(); i++) {
        if (unitVectorAngle(firstnormals->at(i).normal_x,
                            firstnormals->at(i).normal_y,
                            firstnormals->at(i).normal_z,
                            coefficients_first[0],
                            coefficients_first[1],
                            coefficients_first[2]) < firstthres)
        firstfilterind.push_back(i);
    }
  
    PointCloudT::Ptr firstFilter(new PointCloudT(*firstPlane, firstfilterind)); /** Select only points that pass the test */
    PointCloudT::Ptr firstFilterSOR(new PointCloudT); /** Container for after SOR */
  
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(firstFilter);
    sor.setMeanK(30);
    sor.setStddevMulThresh(0.4); /** Small is better for filtering, but too small will filter the good ones */
    sor.filter(*firstFilterSOR);

    /**
     *Perform Euclidean clustering to get rid of floating point cloud chunks
     */   
    PointCloudT::Ptr firstFilterSOREC(new PointCloudT); /** Container for after SOR and EC */

    pcl::search::KdTree<PointT>::Ptr treeec(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(firstFilterSOR);
    tree->nearestKSearch(firstClick, 1, *kIndicesPtr, kDistances);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.04); /** Tolerance in meter(s)... can't be too smaller than pc resolution */
    ec.setMinClusterSize(100);
    ec.setSearchMethod(tree);
    ec.setInputCloud(firstFilterSOR);
    ec.extract(cluster_indices);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    bool is_clicked = false;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            if ((*kIndicesPtr)[0]==*pit) is_clicked = true;
            cloud_cluster->points.push_back(firstFilterSOR->points[*pit]);
        }

        cloud_cluster->width = firstFilterSOR->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        if (is_clicked) { 
            firstFilterSOREC = cloud_cluster;
            break;
        }
    }

    /**
     * Calculate the intersaction line
     */   
    pcl::ModelCoefficients::Ptr intersection_line(new pcl::ModelCoefficients);
    Eigen::Vector3f line;

    /** Find max Y point (closest to CHIMP) */
    float minyval = firstFilterSOREC->at(0).y,
           maxyval = firstFilterSOREC->at(0).y,
           minxval = firstFilterSOREC->at(0).x,
           maxxval = firstFilterSOREC->at(0).x;
    PointT minY(firstFilterSOREC->at(0)),
           maxY(firstFilterSOREC->at(0)),
           minX(firstFilterSOREC->at(0)),
           maxX(firstFilterSOREC->at(0));
    for (int i = 1; i < firstFilterSOREC->size(); i++) {
        if (firstFilterSOREC->at(i).y < minyval) {
            minyval = firstFilterSOREC->at(i).y;
            minY = firstFilterSOREC->at(i);
        }
        
        if (firstFilterSOREC->at(i).y > maxyval) {
            maxyval = firstFilterSOREC->at(i).y;
            maxY = firstFilterSOREC->at(i);
        }
        
        if (firstFilterSOREC->at(i).x < minxval) {
            minxval = firstFilterSOREC->at(i).x;
            minX = firstFilterSOREC->at(i);
        }
        
        if (firstFilterSOREC->at(i).x > maxxval) {
            maxxval = firstFilterSOREC->at(i).x;
            maxX = firstFilterSOREC->at(i);
        }
    }
    
    /** Find clouds that match 25% and 75% distances between the two end points */
    PointT pt1e(maxY), pt2e(minX);
    float val25 = pt1e.x + 0.25 * (pt2e.x-pt1e.x),
           val75 = pt1e.x + 0.75 * (pt2e.x-pt1e.x),
           valthres = 0.005;
    PointCloudT::Ptr band25(new PointCloudT()),
                     band75(new PointCloudT());
    for (int i = 0; i < firstFilterSOREC->size(); i++) {
        if (firstFilterSOREC->at(i).x < val25+valthres &&
            firstFilterSOREC->at(i).x > val25-valthres)
            band25->push_back(firstFilterSOREC->at(i));
        if (firstFilterSOREC->at(i).x < val75+valthres &&
            firstFilterSOREC->at(i).x > val75-valthres)
            band75->push_back(firstFilterSOREC->at(i));
    }

    /** Find points for at the front (maxY) */
    float maxyval25 = band25->at(0).y;;
    PointT maxY25(band25->at(0));
    for (int i = 1; i < band25->size(); i++) {
        if (band25->at(i).y > maxyval25) {
            maxyval25 = band25->at(i).y;
            maxY25 = band25->at(i);
        }
    }
    
    float maxyval75 = band75->at(0).y;;
    PointT maxY75(band75->at(0));
    for (int i = 1; i < band75->size(); i++) {
        if (band75->at(i).y > maxyval75) {
            maxyval75 = band75->at(i).y;
            maxY75 = band75->at(i);
        }
    }

    /** Generate intersection line */
    PointT pt1(maxY25), pt2(maxY75);
    line[0] = pt2.x - pt1.x;
    line[1] = pt2.y - pt1.y;
    line[2] = pt2.z - pt1.z;
    
    intersection_line->values.resize(6);
    intersection_line->values[0] = pt1.x;
    intersection_line->values[1] = pt1.y;
    intersection_line->values[2] = pt1.z;

    intersection_line->values[3] = line[0];
    intersection_line->values[4] = line[1];
    intersection_line->values[5] = line[2];
    
    /**
     *Project points on the line 
     */
    pcl::ProjectInliers<PointT> proj_line; 
    proj_line.setModelType(pcl::SACMODEL_LINE);
    proj_line.setInputCloud(firstFilterSOREC);
    proj_line.setModelCoefficients(intersection_line);
    PointCloudT::Ptr cloud_projected(new PointCloudT);
    proj_line.filter(*cloud_projected); /** Entire input point cloud projected onto this line... */

    Eigen::Vector4f min_pt, max_pt;

    pcl::getMinMax3D(*cloud_projected, min_pt, max_pt);
    float distance1, distance2, distance3;
    distance1 = distL2(max_pt[0], max_pt[1], max_pt[2],
                       min_pt[0], min_pt[1], min_pt[2]);

    PointT line1Mid;
    line1Mid.x = (max_pt[0]+min_pt[0]) / 2;
    line1Mid.y = (max_pt[1]+min_pt[1]) / 2;
    line1Mid.z = (max_pt[2]+min_pt[2]) / 2;
    
    
    PointT vertex1, vertex2;
    float vector_length = sqrt(line[0]*line[0] + line[1]*line[1] + line[2]*line[2]);
    vertex1.x = 0.5 * distance1 * line[0] / vector_length + line1Mid.x;
    vertex1.y = 0.5 * distance1 * line[1] / vector_length + line1Mid.y;
    vertex1.z = 0.5 * distance1 * line[2] / vector_length + line1Mid.z;

    vertex2.x = -0.5 * distance1 * line[0] / vector_length + line1Mid.x;
    vertex2.y = -0.5 * distance1 * line[1] / vector_length + line1Mid.y;
    vertex2.z = -0.5 * distance1 * line[2] / vector_length + line1Mid.z;

    /**
     *Project the first plane on the third plane
     */   
    PointCloudT::Ptr plane1_projected(new PointCloudT);
    PointCloudT::Ptr edge2_projected(new PointCloudT);

    proj_line.setModelType(pcl::SACMODEL_PLANE);
    proj_line.setInputCloud(firstFilterSOREC);
    proj_line.setModelCoefficients(first_plane_visual);
    proj_line.filter(*plane1_projected);
    
    float normalx = line[0] / distL2(line[0], line[1], line[2], 0, 0, 0),
          normaly = line[1] / distL2(line[0], line[1], line[2], 0, 0, 0),
          normalz = line[2] / distL2(line[0], line[1], line[2], 0, 0, 0);
    
    for (int i = 0; i < plane1_projected->points.size(); ++i) {
        float scale = ((plane1_projected->points[i].x - vertex1.x) * normalx +
                       (plane1_projected->points[i].y - vertex1.y) * normaly +
                       (plane1_projected->points[i].z - vertex1.z) * normalz);
        edge2_projected->points.push_back(
            PointT(plane1_projected->points[i].x - scale*normalx,
                   plane1_projected->points[i].y - scale*normaly,
                   plane1_projected->points[i].z - scale*normalz));
    }
    edge2_projected->width = edge2_projected->points.size();
    edge2_projected->height = 1;
    edge2_projected->is_dense = true;
    
    /**
     *Calculate the length of the other edge of the first plane
     */   
    pcl::getMinMax3D(*edge2_projected, min_pt, max_pt);
    distance2  = distL2(max_pt[0], max_pt[1], max_pt[2],
                        min_pt[0], min_pt[1], min_pt[2]);
    
    PointT line2Mid;
    line2Mid.x = (max_pt[0]+min_pt[0]) / 2;
    line2Mid.y = (max_pt[1]+min_pt[1]) / 2;
    line2Mid.z = (max_pt[2]+min_pt[2]) / 2;



    /**
     *Calculate the other six points 
     */
    PointT vertex3;
    PointT vertex4;
    PointT vertex5;
    PointT vertex6;
    PointT vertex7;
    PointT vertex8;
    
    vertex3.x = 2 * (line2Mid.x-vertex1.x) + vertex1.x;
    vertex3.y = 2 * (line2Mid.y-vertex1.y) + vertex1.y;
    vertex3.z = 2 * (line2Mid.z-vertex1.z) + vertex1.z;

    vertex4.x = 2 * (line2Mid.x-vertex1.x) + vertex2.x;
    vertex4.y = 2 * (line2Mid.y-vertex1.y) + vertex2.y;
    vertex4.z = 2 * (line2Mid.z-vertex1.z) + vertex2.z;

    /** Do extrude or use original way */
    float extrude_m  = extrude_in * 0.0254;
    /** Negative since plane normals point towards the origin, but we want it to point away */
    vertex5.x = vertex1.x + -extrude_m*coefficients_first[0];
    vertex5.y = vertex1.y + -extrude_m*coefficients_first[1];
    vertex5.z = vertex1.z + -extrude_m*coefficients_first[2];
       
    vertex6.x = vertex2.x + -extrude_m*coefficients_first[0];
    vertex6.y = vertex2.y + -extrude_m*coefficients_first[1];
    vertex6.z = vertex2.z + -extrude_m*coefficients_first[2];
    
    vertex7.x = vertex3.x + -extrude_m*coefficients_first[0];
    vertex7.y = vertex3.y + -extrude_m*coefficients_first[1];
    vertex7.z = vertex3.z + -extrude_m*coefficients_first[2];
    
    vertex8.x = vertex4.x + -extrude_m*coefficients_first[0];
    vertex8.y = vertex4.y + -extrude_m*coefficients_first[1];
    vertex8.z = vertex4.z + -extrude_m*coefficients_first[2];

    PointT line3Mid;
    line3Mid.x = (vertex1.x+vertex5.x) / 2;
    line3Mid.y = (vertex1.y+vertex5.y) / 2;
    line3Mid.z = (vertex1.z+vertex5.z) / 2;


    /**
     *Printing results
     */
    std::cout << "====================distance of the first edge=================" << std::endl;
    std::cout << distL2(vertex1.x, vertex1.y, vertex1.z, vertex2.x, vertex2.y, vertex2.z) << std::endl; 
    std::cout << "====================distance of the second edge=================" << std::endl;
    std::cout << distL2(vertex1.x, vertex1.y, vertex1.z, vertex3.x, vertex3.y, vertex3.z) << std::endl;
    std::cout << "====================distance of the third edge=================" << std::endl;
    std::cout << distL2(vertex1.x, vertex1.y, vertex1.z, vertex5.x, vertex5.y, vertex5.z) << std::endl;


    

    
    
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
    PointT vertexY = vectorPoint[distanceIdx[1]];

    vectorX[0] = vertexX.x - vertex1.x;
    vectorX[1] = vertexX.y - vertex1.y;
    vectorX[2] = vertexX.z - vertex1.z;
    
    vectorY[0] = vertexY.x - vertex1.x;
    vectorY[1] = vertexY.y - vertex1.y;
    vectorY[2] = vertexY.z - vertex1.z;
    
    vectorX = vectorX.normalized();
    vectorY = vectorY.normalized();

    vectorZ = vectorX.cross(vectorY);

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
    this->gripper_coefficients_->at(0) = line1Mid.x+line2Mid.x-vertex1.x;
    this->gripper_coefficients_->at(1) = line1Mid.y+line2Mid.y-vertex1.y;
    this->gripper_coefficients_->at(2) = line1Mid.z+line2Mid.z-vertex1.z-(line3Mid.z-vertex1.z)+0.05;
    this->gripper_coefficients_->at(3) = quat_palm.w();
    this->gripper_coefficients_->at(4) = quat_palm.x();
    this->gripper_coefficients_->at(5) = quat_palm.y();
    this->gripper_coefficients_->at(6) = quat_palm.z();


    }
}
