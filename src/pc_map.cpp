#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <pcl-1.8/pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>


using namespace std;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tg_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tg(new pcl::PointCloud<pcl::PointXYZ>);


ros::Publisher pub_global_map; 


//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

/*
Esta representación es implementada en el tutorial y se le es suministrada al algoritmo de ICP pero para ello se requiere que la versión de pcl cuente 
con memory.h, las versiones de pcl de ROS no cuenta con esta librería, sin embargo es posible correr el ejemplo sin tener en cuenta esta representación. 
*/
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};


void pairAlign (const PointCloud::Ptr loc_cloud_src, const PointCloud::Ptr loc_cloud_tgt, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (loc_cloud_src);
    grid.filter (*src);

    grid.setInputCloud (loc_cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = loc_cloud_src;
    tgt = loc_cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);  
  // Set the point representation
 // reg.setPointRepresentation (pcl::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (std::abs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();
    
  }

	//
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tg_rgb, *cloud_tg_rgb, targetToSource);

  //add the source to the transformed target
  *cloud_src_rgb += *cloud_tg_rgb;
  //La nube de puntos Source se transfiere a la nube de puntos Target y así se acumulan las nubes de puntos
  pcl::copyPointCloud(*cloud_src_rgb, *cloud_tg_rgb);
  
  //final_transform = targetToSource;
 }

//Se lee la nube de puntos Target, la cual es el objetivo del emparejamiento
 void pc_tg_callback(const sensor_msgs::PointCloud2ConstPtr& cloud){

    pcl::fromROSMsg(*cloud, *cloud_tg_rgb);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_tg_rgb, *cloud_tg_rgb, indices);

}

//Se lee la nube de puntos Source que será alineada con la nube de puntos Target
void pc_src_callback(const sensor_msgs::PointCloud2ConstPtr& cloud){

    pcl::fromROSMsg(*cloud, *cloud_src_rgb);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_src_rgb, *cloud_src_rgb, indices);

    pcl::copyPointCloud(*cloud_src_rgb, *cloud_src);
    pcl::copyPointCloud(*cloud_tg_rgb, *cloud_tg);

    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

    pairAlign(cloud_src, cloud_tg, pairTransform, true);

    pub_global_map.publish(cloud_src_rgb);


}

int main(int argc, char** argv){

    ros::init(argc, argv, "point_cloud_map_generator_node");
    ros::NodeHandle nh;

    pub_global_map = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_map",1);

    ros::Subscriber sub_pc_src = nh.subscribe("catch_point_cloud_src",1,pc_src_callback);
    ros::Subscriber sub_pc_tg = nh.subscribe("catch_point_cloud_tg",1,pc_tg_callback);

    ros::spin();
}
