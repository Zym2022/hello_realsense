#include "hello-realsense.h"

boost::shared_ptr<pcl::visualization::PCLVisualizer> 
simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr template_cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Alignment Result"));
  viewer->setBackgroundColor(0,0,0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> template_color(template_cloud,0,255,0);
  viewer->addPointCloud<pcl::PointXYZ>(template_cloud, template_color, "template cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud,255,0,0);
  viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
  viewer->addCoordinateSystem(0.1);
  // viewer->initCameraParameters();
  viewer->setCameraPosition(0.0, 0.0, -1.0, 0, -1.0, 0.0);
  return (viewer);
}

int main(int argc, char * argv[]) try
{
  // load template file
  std::vector<FeatureCloud> object_templates;
  std::ifstream input_stream(py2var_.template_list_path);
  
  object_templates.resize(0);
  std::string pcd_filename;
  while(input_stream.good())
  {
    std::getline(input_stream,pcd_filename);
    if(pcd_filename.empty()||pcd_filename.at(0) == '#')
      continue;

    FeatureCloud template_cloud;
    template_cloud.loadInputCloud(pcd_filename);
    object_templates.push_back(template_cloud);
  }
  input_stream.close();
  
  if (object_templates.size() < 1)
  {
    printf("No template found!\n");
    return(-1);
  }
  std::cout << "The number of templates is: " << object_templates.size() << std::endl;
  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  for (size_t i = 0;i<object_templates.size();i++)
  {
    template_align.addTemplateCloud(object_templates[i]);
  }

  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud pc;
  // We want the points object to be persistent so we can display the last cloud when a frame drops
  rs2::points points;
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  boost::shared_ptr<rs2::pipeline> pipe(new rs2::pipeline);
  rs2::config cfg;

  if (py2var_.not_from_bag) {
      cfg.enable_stream(RS2_STREAM_COLOR, py2var_.camera_frame_width, py2var_.camera_frame_height, RS2_FORMAT_BGR8, py2var_.camera_fps);
      cfg.enable_stream(RS2_STREAM_DEPTH, py2var_.camera_frame_width, py2var_.camera_frame_height, RS2_FORMAT_Z16,  py2var_.camera_fps);
  }
  else
      cfg.enable_device_from_file(py2var_.bag_path);
      
  // Start streaming with default recommended configuration
  pipe->start(cfg);

  pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis(transformed_cloud, cloud_filtered);
  boost::shared_ptr<pcl::visualization::PCLPlotter> plotter(new pcl::visualization::PCLPlotter);

  while(true) {
    // Wait for the next set of frames from the camera
    auto frames = pipe->wait_for_frames();
    auto depth = frames.get_depth_frame();

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth);
    auto pcl_points = points_to_pcl(points);

    // filter background
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_points);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*cloud_filtered);

    // ... and downsampling the point cloud
    const float voxel_grid_size = 0.003f;
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setInputCloud (cloud_filtered);
    vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
    vox_grid.filter (*cloud_filtered);

    // Assign to the target FeatureCloud
    FeatureCloud target_cloud;
    target_cloud.setInputCloud(cloud_filtered);
    template_align.setTargetCloud(target_cloud);

    TemplateAlignment::Result best_alignment;
    int best_index = template_align.findBestAlignment(best_alignment);
    const FeatureCloud &best_template = object_templates[best_index];
    printf ("Best fitness score: %f\n", best_alignment.fitness_score);

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
    Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

    printf ("\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

    var2py_.best_fitness_score = best_alignment.fitness_score;
    var2py_.rotation_00 = rotation (0,0);
    var2py_.rotation_01 = rotation (0,1);
    var2py_.rotation_02 = rotation (0,2);
    var2py_.rotation_10 = rotation (1,0);
    var2py_.rotation_11 = rotation (1,1);
    var2py_.rotation_12 = rotation (1,2);
    var2py_.rotation_20 = rotation (2,0);
    var2py_.rotation_21 = rotation (2,1);
    var2py_.rotation_22 = rotation (2,2);
    var2py_.translation_x = translation (0);
    var2py_.translation_y = translation (1);
    var2py_.translation_z = translation (2);

    pcl::transformPointCloud (*best_template.getPointCloud (), *transformed_cloud, best_alignment.final_transformation);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> template_color(transformed_cloud, 0 , 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud.getPointCloud(), 255, 0, 0);
    viewer->updatePointCloud<pcl::PointXYZ>(target_cloud.getPointCloud(), target_color, "target cloud");
    viewer->updatePointCloud<pcl::PointXYZ>(transformed_cloud, template_color, "template cloud");
    viewer->spinOnce(100);
  }
  return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
  std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
}
catch (const std::exception & e)
{
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
