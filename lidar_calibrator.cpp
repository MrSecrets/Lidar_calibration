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
#include <math.h>  
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

/*std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > sourceClouds(100);*/

using namespace std;
using namespace cmath;

namespace calibrator{
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

POINT_CLOUD_REGISTER_POINT_STRUCT(clustering::PointSphere,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, r,r)
                                  (float, theta, theta)
                                  (float, phi, phi)
                                  (float, sigma1, sigma1)
                                  (float, sigma2, sigma2)
                                  (float, sigma3, sigma3)
                                  (int, ring, ring)
                                  )

#define PointSphere clustering::PointSphere


class lidar_calibrator{
	public:
		lidar_calibrator(){
			points_sub = nh.subscribe("/ouster_points", 1, &lidar_calibrator::lidar_callback,this);
		}
	private:
		ros::NodeHandle nh;
		ros::Subscriber points_sub;
		ros::Publisher sigma_pub;
		
		void lidar_callback(const pcl::PointCloud& cloud);
		void estimate_planarity();
		void estimate_planarity_single();
		double a2d();
		double energy();
		double optimization();
		double transformMatrix();

};


int main(int argc, char **argv)
{

    ros::init(argc, argv,"lidar_calibrator_node");
    lidar_calibrator node;
    ros::spin();
    return 0;
}

void lidar_calibrator::estimate_planarity(/* &pointSphere, R, Cloud */)
{	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);
  
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
	    	PointXYZ Sphere;

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

void lidar_calibrator::estimate_planarity_single(const pcl::PointCloud& cloud, R, Cloud, PointSphere rPoint )
{	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);
	
  float radius = 1;
  // ut all these in a for leap iterating over every point
  vector<int> pointIdxRadiusSearch;
  vector<float> pointRadiusSquaredDistance;
  pcl::PointCloud<PointXYZ> SearchPoint;
	SearchPoint.x = rPoint.x;
	SearchPoint.y = rPoint.y;
	SearchPoint.z = rPoint.z;
  int point_number;
		if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
			PointSphere point;

			point.x = cloud->points[ pointIdxRadiusSearch[i] ].x ;
			point.y = cloud->points[ pointIdxRadiusSearch[i] ].y ;
			point.z = cloud->points[ pointIdxRadiusSearch[i] ].z ;
			SearchPoint.push_back(point);
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
		rpoint.x = searchPoint.x;
		rpoint.y = searchPoint.y;
		rpoint.z = searchPoint.z;
		rpoint.sigma1 = sqrt(abs(eigvec[0]));
		rpoint.sigma2 = sqrt(abs(eigvec[1]));
		rpoint.sigma3 = sqrt(abs(eigvec[2]));
}

double lidar_calibrator::a2d(/*point1(p),point2(m)*/)
{
	auto a2d1 = (point1.sigma2-point1.sigma3)/point1.sigma1;
	estimate_planarity_single(/*entire point cloud*/, point2)
	decltype(a2d1) a2d2 = (point2.sigma2-point2.sigma3)/point2.sigma1;
	decltype(a2d1) ad = max(a2d1,a2d2);
	return ad
}

double lidar_calibrator::energy(/*pointSphere*/)
{
	//a2d = (sigma2-sigma3)/sigma1;
	vector<PointCloud<PointSphere>::Ptr, Eigen::aligned_allocator <PointCloud <PointSphere>::Ptr> > ringClouds(B); // make an array of pointclouds with number of elemnts = number of rings
	for (int i = 0; i < cloud.size; ++i)
	{
		int ring_id = cloud[i].R;
		cloud[i] -> ringClouds[ring_id].push_back() // the point is appended onto the pointcloud on the i th element of the array.
	}

	auto Jn = 0;
	auto Jd = 0;
	int N = 2;/* # of neighbouring beams to align each beam to */
	// P = pointcloud after extracting the planarity: pointcloud afer passing through the previous function 

	for(int bi = 1; b<=64; bi++)
	{	
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud (rinClouds[bi]);
		for(int bj = bi-N; bj<=bi+N;bj++)  // add break clause in this loop
		{	if(bj<1 or bj<64)continue;
			for(auto k = 0; k <= ringClouds[bj].size(); k++)
			{
				int K = 1;
				vector<int> mk(K);
				vector<float> pointNKNSquaredDistance(K);
				
				PointSphere pk;
				pk.phi = ringClouds[bi][1].phi;
				pk.r = ringClouds[bj][k].r*(sin(abs(ringClouds[bi][1].phi-ringClouds[bj][1].phi)));
				pk.theta = ringClouds[bj][k].theta
				pk.x = ringClouds[bj][k].r*cos(ringClouds[bj][k].theta)*cos(ringClouds[bi][1].phi);
				pk.y = -ringClouds[bj][k].r*sin(ringClouds[bj][k].theta)*cos(ringClouds[bi][1].phi);
				pk.z = ringClouds[bj][k].r*sin(ringClouds[bi][1].phi);
				
				if ( kdtree.nearestKSearch (pk, K, mk, pointNKNSquaredDistance) > 0 )
				{
					if (pointNKNSquaredDistance[0] < dm)   // pointNKNsSquareDistance is dk/nk
					{
						Jn = Jn + a2d(pk,ringClouds[bi][mk[0]])*((ringClouds[bi][k].r) * tan(abs(ringClouds[bi][k].phi-ringClouds[bj][k].phi)) * (pointNKNSquaredDistance[0]))^2;
						Jd = Jd + a2d(pk,ringClouds[bi][mk[0]]);
					}
				}  
			}
		}
	}
	return Jn/Jd ;
}
void lidar_calibrator::optimization(const pcl::PointCloud& cloud,const EigenBase<Derived>& R, const EigenBase<Derived>& T) //find hyperparameters and return final point cloud in global reference frame*/)
{
	i=0;
	if (i==0){
	Eigen::MatrixXf R(3,1)=Eigen:: EMatrixnf::Random(3,1);
	Eigen::MatrixXf T(3,1)=Eigen::Matrixnf::Random(3,1);}// intialise randomly R AND T
	d1 = transformMatrix(Sensorpc,R,T);//returns value of energy function at corresponding R,T,sensorpc
	Cn=0; Dn=0;
	for(int bi =0; bi<64; bi++)  // add break clause in this loop
		{
			for (int bj = bi-N; bj<=bi+N;bj++){
				if(bj<0 or bj>63)continue;
			for(auto k = 0; k<= /*number of points*/; k++)  // i think we should use while loop here
			{	
				Eigen::MatrixXf C(6,1)=Eigen::MatrixXf::Random(6,1);
				Eigen::MatrixXf B(3,1)=Eigen::MatrixXf::Zero(3,1); B(0,0)=1.0;
				Eigen::MatrixXf V(3,1)=Eigen::MatrixXf::Zero(3,1); V(1,0)=1.0;
				Eigen::MatrixXf N(3,1)=Eigen::MatrixXf::Zero(3,1);N(2,0)=1.0;
				//include n 
				Eigen::MatrixXf B1(3,1)=Eigen::MatrixXf::Zero(3,1); B1(0,0)=R(0,0);
				Eigen::MatrixXf V1(3,1)=Eigen::MatrixXf::Zero(3,1); V1(1,0)=R(1,0);
				Eigen::MatrixXf N1(3,1)=Eigen::MatrixXf::Zero(3,1);N1(2,0)=R(2,0);
				//include n 
				// include Rnav and Tnav

				C(0,0)= (n[bj].transpose()*(B)); 
				C(1,0)= (n[bj].transpose()*(V));
				C(2,0)= (n[bj].transpose()*(N));
				B(0,0)=R(0,0); V(1,0)=R(1,0); N(2,0)=R(2,0);
				//include m in the code

				C(3,0)=(n[bj].transpose()*(B1.cwiseProduct(p[bj][k])-B1.cwiseProduct(m[o][k])));
				C(4,0)=(n[bj].transpose()*(V1.cwiseProduct(p[bj][k])-V1.cwiseProduct(m[o][k])));
				C(5,0)=(n[bj].transpose()*(N1.cwiseProduct(p[bj][k])-N1.cwiseProduct(m[o][k])));
				Eigen::MatrixXf D(3,1)= Eigen::MatrixXf::Random(3,1);
				D(0,0)= 0;
				D(1,0)=(R.cwiseProduct(p[bj][k])+T);
				D(2,0)=(R.cwiseProduct(m[o][k])+T);
				//include W

					Cn= Cn + W*(C*(C.transpose())); //to do inclution of w
					Dn= Dn + W*D*C;
					}  }
				}//finds dx,dy,dz,dalpha,dbeta,dgama
		 Eigen::MatrixXf dx(6,1);
		 dx = C.inverse()*V;
		Eigen::MatrixXf G(3,1)= Eigen::Zero(3,1);
		for(int c=0;c<3;c++){
			G(c,0)=R(c,0);
			R= R + G*dx(c+3,0);
			G(c,0)=T(c,0);
			T= T+ G*dx(c,0);
			G(c,0)=0;
			}// update R and T 
		d2= transformMatrix(Sensorpc,R,T)
		if (d2-d1>(-1*h) and d2-d1<h){

				return;
			}

		else{
	 		optimization(ptr,R,T);
		}

		i=i+1;
	}//recursion condition


double lidar_calibrator::transformMatrix(const pcl::PointCloud& cloud,const EigenBase<Derived>& R, const EigenBase<Derived>& T)
{
//converts x,y,z from sensor to global reference plane and returns value of energy function	
	vector<PointCloud<PointSphere>::Ptr, Eigen::aligned_allocator <PointCloud <PointSphere>::Ptr> > ringClouds(B); // make an array of pointclouds with number of elemnts = number of rings
	for (int i = 0; i < cloud.size; ++i)
	{
		int ring_id = SensorPC[i].R;
		SensorPC[i] -> ringClouds[ring_id].push_back(); // the point is appended onto the pointcloud on the i th element of the array.
	}
	
	pcl::PointCloud<PointSphere> GlobalPC;
	N = 2;
	
	for(int bi =0; bi<=64; bi++)  // add break clause in this loop
	{
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud (rinClouds[bi]);

		for (int bj = bi-N; bj<=bi+N;bj++)
		{
			for(auto k = 0; k<= ringClouds[bj].size(); k++)
			{
				Eigen::MatrixXf Z(3,1);Eigen::MatrixXf Y(3,1);
				Z(0,0)= ringClouds[bi][k].x; 
				Z(1,0)= ringClouds[bi][k].y;
				Z(1,1)= ringClouds[bi][k].z;
				Y(0,0)=	ringClouds[bj][k].x;
				Y(1,0)= ringClouds[bj][k].y;
				Y(1,1)= ringClouds[bj][k].z;
				//include m(ptr) and m1(globalpc)
				int K = 1;
				vector<int> mk(K);
				vector<float> pointNKNSquaredDistance(K);
				
				PointSphere pk;
				pk.phi = ringClouds[bi][1].phi;
				pk.r = ringClouds[bj][k].r*(sin(abs(ringClouds[bi][1].phi-ringClouds[bj][1].phi)));
				pk.theta = ringClouds[bj][k].theta;
				pk.x = ringClouds[bj][k].r*cos(ringClouds[bj][k].theta)*cos(ringClouds[bi][1].phi);
				pk.y = -ringClouds[bj][k].r*sin(ringClouds[bj][k].theta)*cos(ringClouds[bi][1].phi);
				pk.z = ringClouds[bj][k].r*sin(ringClouds[bi][1].phi);
				
				if ( kdtree.nearestKSearch (pk, K, mk, pointNKNSquaredDistance) > 0 )
				{	
					// include Rnav and Tnav
					Z = (R.cwiseProduct(Z)+T);
					Y = (R.cwiseProduct(Y)+T);
					pcl::PointCloud<PointSphere> pointtemp; 
					pointtemp.x = Z(0,0);
					pointtemp.y = Z(1,0);
					pointtemp.z = Z(2,0);
					pointtemp.r = sqrt(pow(Z(0,0),2)+pow(Z(1,0),2)+pow(Z(2,0),2));
					pointtemp.theta = atan((sqrt(pow(Z(0,0),2)+pow(Z(1,0),2))/Z(2,0))-90);
					pointtemp.phi = atan(Z(1,0)/Z(0,0));
					pointtemp.ring = bj;

					GlobalPC.pushback(pointtemp);
				}  
			}  
		}
	}
	return energy(GlobalPC);
}
void lidar_callback(const pcl::PointCloud& cloud) 
{
 input = cloud; // copy the variable that the callback passes in to you class variable (attribute) input

}
