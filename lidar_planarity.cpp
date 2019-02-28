
#include <iostream>
//#include <home/sine/Downloads/pcl-pcl-1.8.0/gpu/features/include/pcl/gpu/features/features.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#define PCL_NO_PRECOMPILE
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
//#include <pcl/ros/conversions.h>
#include <velodyne_pointcloud/point_types.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

/*std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > sourceClouds(100);*/

using namespace std;

namespace planarity{
  struct PointXYZQWE  // defines a  pointcloud that contains x y z and q w e (i.e sigma1, sigma2, sigma3)
  {
    PCL_ADD_POINT4D;                    
    float sigma1;                 
    float sigma2;                      
    float sigma3;                     
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     
  } EIGEN_ALIGN16;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(clustering::PointXYZIRL,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, sigma1, sigma1)
                                  (float, sigma2, sigma2)
                                  (float, sigma3, sigma3))

#define PlanarityPointXYZQWE planarity::PointXYZQWE


class planarity_extractor{
	public:
		planarity_extractor(){
			points_sub = nh.subscribe("/velodyne_points", 1, &plane_fit::velodyne_callback,this);
		}
	private:
		ros::NodeHandle nh;
		ros::Subscriber points_sub;
		ros::Publisher sigma_pub;
		
		void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

		}
		void estimate_planarity(void);
};


int main(int argc, char **argv)
{

    ros::init(argc, argv,"planarity_extractor_node");
    planarity_extractor node;
    ros::spin();
    return 0;
}

void estimate_planarity(/* &pontXYZQWE, R, Cloud */){	
  float radius = 1;
  // ut all these in a for leap iterating over every point
  vector<int> pointIdxRadiusSearch;
  vector<float> pointRadiusSquaredDistance;
  pcl::PointCloud<PointXYZ> rSearchPoint;
  int point_number;
  point_number = cloud.size();
  for ( iterate = 0; i<point_number; ++iterate)
  {
	    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
	  {
	    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
	    	PointXYZ point;

	      point.x = cloud->points[ pointIdxRadiusSearch[i] ].x ;
	      point.y = cloud->points[ pointIdxRadiusSearch[i] ].y ;
	      point.z = cloud->points[ pointIdxRadiusSearch[i] ].z ;
	      rSearchPoint.push_back(point);
	  }
	  	rSearchPoint.push_back(searchPoint);

	  	pcl::compute3DCentroid(*rSearchPoint, centroid); // check this function
	  	// define matrix M = (x1 - cenroid ..... xn - centroid)
	  	n = rSearchPoint.size();
	  	Eigen::Vector3f M;
	  	for( i = 0; i<=n;i++){
	  		M.push_back (rSearchPoint[i] - centroid);
	  	}
	  	
	  	Eigen::Matrix3f lambda;
	  	// M'M/n = RyR'

	  	lambda = (R.inverse()*M.transpose()*M*(R.transpose()).inverse())/n;  // lambda is supposed to be a digonal matrix
	  	Eigen::Vector3d eigvec = lambda.diagonal();
	  	sort(eigvec.data(),eigvec.data()+eigvec.size());  //sorting in descnding order
	  	// adding all thesse values onto a pointcloud, the one we previously defined
	  	planarity_cloud[iterate].x = searchPoint.x;
	  	planarity_cloud[iterate].y = searchPoint.y;
	  	planarity_cloud[iterate].z = searchPoint.z;
	  	planarity_cloud[iterate]sigma1 = sqrt(abs(eigvec[0]));
	  	planarity_cloud[iterate]sigma2 = sqrt(abs(eigvec[1]));
	  	planarity_cloud[iterate]sigma3 = sqrt(abs(eigvec[2])); 
	  }
}

double energy(/*pointcloud xyzr */){
	double J = 0;
	int B = /* # of beams */;
	int N = /* # of neighbouring beams to align each beam to */;
	// P = pointcloud after extracting the planarity: pointcloud afer passing through the previous function 

		for(bi = 1; b<=B; bi++)
		{
			for(bj = bi-N; bj<=bi+N;bj++)  // add break clause in this loop
			{
				for(k = 0; k<= number of beams; k++)  // i think we should use while loop here
				{
					if (abs(pk - mk) > dm)   
					 {
					 	J = J + (nk*(pk - mk))^2; ? // nk is surface normal
					 } 
				}
			}
		}

	return J ;
}

// Kd Search
// 3D centroid
// eigen calculations