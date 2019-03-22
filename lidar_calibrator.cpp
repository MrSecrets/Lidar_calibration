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
#include <pcl/ros/conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <cmath>
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

/*std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > sourceClouds(100);*/

using namespace std;
using namespace cmath;

namespace planarity{
  struct PointSphere  // defines a  pointcloud that contains x y z, r theta phi and sigma1 sigma2 sigma3
  {
    PCL_ADD_POINT4D;
    float r;
    float theta;
    float phi;                    
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
                                  (float, r,r)
                                  (float, theta, theta)
                                  (float, phi, phi)
                                  (float, sigma1, sigma1)
                                  (float, sigma2, sigma2)
                                  (float, sigma3, sigma3))

#define PlanarityPointXYZQWE planarity::PointXYZQWE


class lidar_calibrator{
	public:
		lidar_calibrator(){
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

    ros::init(argc, argv,"lidar_calibrator_node");
    lidar_calibrator node;
    ros::spin();
    return 0;
}

void estimate_planarity(/* &pointSphere, R, Cloud */)
{	
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
	  	planarity_cloud[iterate].sigma1 = sqrt(abs(eigvec[0]));
	  	planarity_cloud[iterate].sigma2 = sqrt(abs(eigvec[1]));
	  	planarity_cloud[iterate].sigma3 = sqrt(abs(eigvec[2]));
	  }
}

double a2d(/*point1,point2*/)
{
	auto a2d1 = (point1.sigma2-point1.sigma3)/point1.sigma1;
	decltype(a2d1) a2d2 = (point2.sigma2-point2.sigma3)/point2.sigma1;
	decltype(a2d1) ad = max(a2d1,a2d2);
	return ad
}

double energy(/*pointSphere*/)
{
	//a2d = (sigma2-sigma3)/sigma1;
	int B = 64; /* # of beams */; // the length of the vector/array should be equal to B
	vector<PointCloud<PointXYZ>::Ptr, Eigen::aligned_allocator <PointCloud <PointXYZ>::Ptr> > ringClouds(B); // make an array of pointclouds with number of elemnts = number of rings
	for (int i = 0; i < cloud.size; ++i)
	{
		int ring_id = cloud[i].R;
		cloud[i] -> ringClouds[ring_id].push_back() // the point is appended onto the pointcloud on the i th element of the array.
	}

	auto Jn = 0;
	auto Jd = 0;
	int N = 2;/* # of neighbouring beams to align each beam to */
	// P = pointcloud after extracting the planarity: pointcloud afer passing through the previous function 

	for(int bi = 1; b<=B; bi++)
	{
		for(int bj = bi-N; bj<=bi+N;bj++)  // add break clause in this loop
		{
			for(auto k = 0; k<= /*number of beams*/; k++)  // i think we should use while loop here
			{
				auto dk = ((ringClouds[bi][k].r)*tan(abs(ringClouds[bi][k].phi-ringClouds[bj][k].phi))     //we assume that the matrix is the the  
									*(ringClouds[bj][k].r)*tan(abs(ringClouds[bi][k].phi-ringClouds[bj][k].phi)));	//already transformed matix ie it is already pk
				Jn = Jn + a2d()*dk^2;  // nk is surface normal nk = normal_Cloud[k]
				Jd = Jd + a2d();	  
			}
		}
	}
	return Jn/Jd ;
}