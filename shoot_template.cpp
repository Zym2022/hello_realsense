#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <thread>
#include <boost/thread/thread.hpp>
#include <time.h>
#include <string>


pcl::PointCloud<pcl::PointXYZ>::Ptr 
points_to_pcl(const rs2::points& points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }
    return cloud;
}

float background_depth = 1.0;
float background_left = -0.2;
float background_right = 0.2;
float background_up = -0.2;
float background_down = 0.2;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event)
{
    // ----------------z-axis---------------------
    if (event.getKeySym () == "a" && event.keyDown ())
    {
    background_depth -= 0.1;
    std::cout << "a was pressed => filter points deeper than " << background_depth << std::endl;
    }

    if (event.getKeySym () == "s" && event.keyDown ())
    {
    background_depth -= 0.01;
    std::cout << "s was pressed => filter points deeper than " << background_depth << std::endl;
    }

    if (event.getKeySym () == "d" && event.keyDown ())
    {
    background_depth += 0.01;
    std::cout << "d was pressed => filter points deeper than " << background_depth << std::endl;
    }

    if (event.getKeySym () == "f" && event.keyDown ())
    {
    background_depth += 0.1;
    std::cout << "d was pressed => filter points deeper than " << background_depth << std::endl;
    }

    // ----------------x-axis-------------------- 
    if (event.getKeySym () == "w" && event.keyDown ())
    {
    background_left -= 0.02;
    std::cout << "w was pressed => filter points lefter than " << background_left << std::endl;
    }

    if (event.getKeySym () == "e" && event.keyDown ())
    {
    background_left += 0.02;
    std::cout << "e was pressed => filter points lefter than " << background_left << std::endl;
    }

    if (event.getKeySym () == "z" && event.keyDown ())
    {
    background_right -= 0.02;
    std::cout << "z was pressed => filter points righter than " << background_right << std::endl;
    }

    if (event.getKeySym () == "x" && event.keyDown ())
    {
    background_right += 0.02;
    std::cout << "x was pressed => filter points righter than " << background_right << std::endl;
    }

    // ----------------y-axis---------------------
    if (event.getKeySym () == "r" && event.keyDown ())
    {
    background_up -= 0.02;
    std::cout << "r was pressed => filter points higher than " << background_up << std::endl;
    }

    if (event.getKeySym () == "t" && event.keyDown ())
    {
    background_up += 0.02;
    std::cout << "t was pressed => filter points higher than " << background_up << std::endl;
    }

    if (event.getKeySym () == "v" && event.keyDown ())
    {
    background_down -= 0.02;
    std::cout << "x was pressed => filter points lower than " << background_down << std::endl;
    }

    if (event.getKeySym () == "b" && event.keyDown ())
    {
    background_down += 0.02;
    std::cout << "c was pressed => filter points lower than " << background_down << std::endl;
    }


    // -----------------save--------------------
    if (event.getKeySym () == "p" && event.keyDown ())
    {
    time_t timep;
    struct tm *p;
    time(&timep);
    p = localtime(&timep);

    std::string pcd_name;
    pcd_name = std::to_string(1900 + p->tm_year) + "_" + std::to_string(1 + p->tm_mon) + "_" +
                std::to_string(p->tm_mday) + "_" + std::to_string(p->tm_hour) + "_" + 
                std::to_string(p->tm_min) + "_" + std::to_string(p->tm_sec) + ".pcd";

    std::cout << "p was pressed => save current pcd " << std::endl;
    pcl::io::savePCDFileBinary("/home/zju/realsense_ws/templete_pcd/" + pcd_name, *cloud_filtered);
    }
}


void mouseEventOccurred (const pcl::visualization::MouseEvent &event)
{
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;
  }
}

pcl::visualization::PCLVisualizer::Ptr interactionCustomizationVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud)
{

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (0.1);

    viewer->registerKeyboardCallback (keyboardEventOccurred);

    viewer->registerMouseCallback (mouseEventOccurred);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud,255,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    
    viewer->setCameraPosition(0.0, 0.0, -1.0, 0, -1.0, 0.0);

    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> 
simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Alignment Result"));
    viewer->setBackgroundColor(0,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud,255,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    viewer->addCoordinateSystem(0.1);
    //   viewer->initCameraParameters();
    viewer->setCameraPosition(0.0, 0.0, -1.0, 0, -1.0, 0.0);
    return (viewer);
}

int main(int argc, char * argv[])
{
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    boost::shared_ptr<rs2::pipeline> pipe(new rs2::pipeline);
    rs2::config cfg;

    const int camera_frame_width  = 848;
    const int camera_frame_height = 480;
    const int camera_fps          = 30;

    cfg.enable_stream(RS2_STREAM_COLOR, camera_frame_width, camera_frame_height, RS2_FORMAT_BGR8, camera_fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, camera_frame_width, camera_frame_height, RS2_FORMAT_Z16,  camera_fps);

    // Start streaming with default recommended configuration
    pipe->start(cfg);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = interactionCustomizationVis(cloud_filtered);

    while (true)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe->wait_for_frames();
        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);
        auto pcl_points = points_to_pcl(points);

        // filter background
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(pcl_points);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(0.0, background_depth);
        pass_z.filter(*cloud_filtered);

        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(cloud_filtered);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(background_left, background_right);
        pass_x.filter(*cloud_filtered);

        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(cloud_filtered);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(background_up, background_down);
        pass_y.filter(*cloud_filtered);

        // ... and downsampling the point cloud
        const float voxel_grid_size = 0.003f;
        pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
        vox_grid.setInputCloud (cloud_filtered);
        vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
        vox_grid.filter (*cloud_filtered);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_filtered ,255, 0, 0);
        viewer->updatePointCloud<pcl::PointXYZ>(cloud_filtered, target_color, "target cloud");
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));

    }
}
