#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>

#include <thread>
#include <time.h>
#include <tbb/parallel_for.h>

const int camera_frame_width  = 848;
const int camera_frame_height = 480;
const int camera_fps          = 30;

// Struct for managing rotation of pointcloud view
struct state {
    state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
        ml(false), offset_x(0.0f), offset_y(0.0f) {}
    double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y; 
};

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

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

class FeatureCloud
{
    public:
      // rename some class
      typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
      typedef pcl::PointCloud<pcl::Normal> SurfaceNormal;
      typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
      typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

      //类内变量用下划线连接，并用下划线结尾
      //构造函数
      FeatureCloud():
        search_method_xyz_(new SearchMethod),
        normal_radius_(0.02f),
        feature_radius_(0.02f)
      {}

      //析构函数
      ~FeatureCloud(){}

      //设置点云的输入方式
      //直接输入一个点云指针
      void
      setInputCloud(PointCloud::Ptr xyz)
      {
        xyz_ = xyz;
        processInput();
      }

      //从pcd文件中加载点云
      void
      loadInputCloud(const std::string &pcd_file)
      {
        xyz_ = PointCloud::Ptr(new PointCloud);
        pcl::io::loadPCDFile(pcd_file, *xyz_);
        processInput();
      }

      //定义数据存取方法
      //读取指向点云的指针
      PointCloud::Ptr
      getPointCloud() const
      {
        return (xyz_);
      }

      //读取指向点云法线的指针
      SurfaceNormal::Ptr
      getSurfaceNormals() const
      {
        return (normals_);
      }

      //获取指向点云特征描述子的指针
      LocalFeatures::Ptr 
      getLocalFeatures() const
      {
        return (features_);
      }

    protected:
      //处理输入点云
      //计算点云曲面法线，估计曲面特征描述子
      void
      processInput()
      {
        computeSurfaceNormals();
        computerLocalFeatures();
      }

      void 
      computeSurfaceNormals()
      {
        normals_ = SurfaceNormal::Ptr(new SurfaceNormal);

        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
        norm_est.setInputCloud(xyz_);
        norm_est.setSearchMethod(search_method_xyz_);
        norm_est.setRadiusSearch(normal_radius_);
        norm_est.compute(*normals_);
      }

      void 
      computerLocalFeatures()
      {
        features_ = LocalFeatures::Ptr(new LocalFeatures);

        pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
        fpfh_est.setInputCloud(xyz_);
        fpfh_est.setInputNormals(normals_);
        fpfh_est.setSearchMethod(search_method_xyz_);
        fpfh_est.setRadiusSearch(feature_radius_);
        fpfh_est.compute(*features_);
      }
    
    private:
      //数据变量
      PointCloud::Ptr xyz_;
      SurfaceNormal::Ptr normals_;
      LocalFeatures::Ptr features_;
      SearchMethod::Ptr search_method_xyz_;

      //参数变量
      float normal_radius_;
      float feature_radius_;
};


class TemplateAlignment
{
    public:
      //创建一个储存匹配结果的结构
      struct Result
      {
        float fitness_score;
        Eigen::Matrix4f final_transformation;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      };

      TemplateAlignment():
        min_sample_distance_(0.05f),
        max_correspondence_distance_(0.01f*0.01f),
        nr_iterations_(500)
        {}
      
      ~TemplateAlignment(){}

      // Set the given cloud as the target to which the templates will be aligned
      void
      setTargetCloud(FeatureCloud &target_cloud)
      {
        target_ = target_cloud;
      }

      // Add the given cloud to the list of template clouds
      void
      addTemplateCloud(FeatureCloud &template_cloud)
      {
        templates_.push_back(template_cloud);
      }

      // Align the given template cloud to the target specified by setTargetCloud ()
      void 
      align(FeatureCloud &template_cloud, TemplateAlignment::Result &result)
      {
        pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
        sac_ia.setMinSampleDistance(min_sample_distance_);
        sac_ia.setMaxCorrespondenceDistance(max_correspondence_distance_);
        sac_ia.setMaximumIterations(nr_iterations_);

        sac_ia.setInputTarget(target_.getPointCloud());
        sac_ia.setTargetFeatures(target_.getLocalFeatures());
        sac_ia.setInputCloud(template_cloud.getPointCloud());
        sac_ia.setSourceFeatures(template_cloud.getLocalFeatures());

        pcl::PointCloud<pcl::PointXYZ> registration_output;
        sac_ia.align(registration_output);

        result.fitness_score = (float) sac_ia.getFitnessScore(max_correspondence_distance_);
        result.final_transformation = sac_ia.getFinalTransformation();
      }

      // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
      void 
      alignAll(std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result>> &results)
      {
        results.resize(templates_.size());
        tbb::parallel_for(0, int(templates_.size()), [&](int i)
        {
          align(templates_[i], results[i]);
        });
      }

      // Align all of template clouds to the target cloud to find the one with best alignment score
      int
      findBestAlignment(TemplateAlignment::Result &result)
      {
        // Align all of the templates to the target cloud
        std::vector<Result, Eigen::aligned_allocator<Result>> results;
        alignAll(results);

        // Find the template with the best (lowest) fitness score
        float lowest_score = std::numeric_limits<float>::infinity();
        int best_template = 0;
        for(size_t i = 0;i<results.size();i++)
        {
          const Result &r = results[i];
          if(r.fitness_score<lowest_score)
          {
            lowest_score = r.fitness_score;
            best_template = (int) i;
          }
        }

        result = results[best_template];
        return (best_template);
      }

      int
      findAlignment(TemplateAlignment::Result &result)
      {
        align(templates_[0], result);
        return 0;
      }

    private:
      // A list of template clouds and the target to which they will be aligned
      std::vector<FeatureCloud> templates_;
      FeatureCloud target_;

      // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
      pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
      float min_sample_distance_;
      float max_correspondence_distance_;
      int nr_iterations_;      
};