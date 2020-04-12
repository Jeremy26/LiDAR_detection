// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>());
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-.4,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);  

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point:indices)
        inliers->indices.push_back(point);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
  typename  pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
  for (int index:inliers->indices)
  planeCloud->points.push_back(cloud->points[index]);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size()==0)
    {
        std::cout<<"Could not estimate a plane"<<std::endl;
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;

  srand(time(NULL));
  
  // TODO: Fill in this function
  while (maxIterations--) {
    
    // Add 3 unique points at random
    std::unordered_set<int> curr_inliers; 
    while (curr_inliers.size() != 3) {
      curr_inliers.insert(rand() % cloud->points.size());
    }

    auto itr = curr_inliers.begin();
    float x1 = cloud->points[*itr].x;
    float y1 = cloud->points[*itr].y;
    float z1 = cloud->points[*itr].z;
    itr++;
    float x2 = cloud->points[*itr].x;
    float y2 = cloud->points[*itr].y;
    float z2 = cloud->points[*itr].z;
    itr++;
    float x3 = cloud->points[*itr].x;
    float y3 = cloud->points[*itr].y;
    float z3 = cloud->points[*itr].z;

    // plane params
    float a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
    float b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
    float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
    float d = - (a*x1 + b*y1 + c*z1);

    // Measure distance between every point and fitted line
    for (int i = 0; i < cloud->points.size(); ++i)
    {
      if (curr_inliers.count(i) > 0)
        continue;

      float nx = cloud->points[i].x;
      float ny = cloud->points[i].y;
      float nz = cloud->points[i].z;
      float dist = fabs(a * nx + b * ny + c * nz + d) / sqrt(a * a + b * b + c * c);

      if (dist < distanceTol)
        curr_inliers.insert(i);
    }

    if (curr_inliers.size() > inliersResult.size())
      inliersResult = curr_inliers;

  }
    // Using ObstCloud-planeCloud as inliers-outliers
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    //Short alternative to separateClouds function from unordered_int
    for (int i=0; i<cloud->points.size();i++)
    {
        PointT point = cloud->points[i];
        if (inliersResult.count(i))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

  //Return the pair from unordered_int
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers,cloudInliers);
  
  return segResult;

}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<int>& cluster, std::vector<bool> &processed, KdTree *tree, float distanceTol)
{
	processed[indice]=true;
	cluster.push_back(indice);
	std::vector<int> nearest = tree->search(cloud->points[indice], distanceTol);
	for (int id:nearest)
	{
		if(!processed[id]){
			clusterHelper(id, cloud, cluster,processed,tree, distanceTol);
		}
	}

}
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters; // Actual return element

	//std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(cloud->points.size(),false);
	int i=0;
	while (i<cloud->points.size())
	{
    //CASE WHERE POINT HAS ALREADY BEEN PROCESSED
		if(processed[i])
		{
			i++;
			continue;
		}
  
    // CASE WHERE POINT HASN'T BEEN PROCESSED YET
    std::vector<int> cluster;
		clusterHelper(i, cloud, cluster, processed, tree, distanceTol);
    typename pcl::PointCloud<PointT>::Ptr finalCluster(new pcl::PointCloud<PointT>); // Part of the return vector

    // setMinSize() and setMaxSize() replacement
  	if (cluster.size()>=minSize && cluster.size()<=maxSize)
    {
      for (int j=0; j<cluster.size();j++)
      {
        finalCluster->points.push_back(cloud->points[cluster[j]]); // Add every element to the cluster

      }
      finalCluster->is_dense = true;
      finalCluster->width = finalCluster->points.size();
      finalCluster->height = 1;
		  clusters.push_back(finalCluster); // Add cluster to the list of clusters
      i++;
    }

	}
	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);
    for (pcl::PointIndices getIndices: clusterIndices)
  {
    typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
    for (int index: getIndices.indices)
            cloudCluster->points.push_back (cloud->points[index]);
    cloudCluster->width = cloudCluster->points.size ();
    cloudCluster->height = 1;
    cloudCluster->is_dense = true;

    clusters.push_back(cloudCluster);
  }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}