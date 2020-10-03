#include <kinectfusion.h>
#include <depth_camera.h>
#include <util.h>

#include <iostream>
#include <fstream>
#include <iomanip>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Weffc++"
#include <opencv2/highgui.hpp>
#pragma GCC diagnostic pop

#include <cxxopts.hpp>
#include <cpptoml.h>

// ILLIXR includes
#include "common/phonebook.hpp"
#include "common/plugin.hpp"
#include "common/threadloop.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"

std::string data_path {};
std::string recording_name {};

using namespace ILLIXR;

auto make_configuration(const std::shared_ptr<cpptoml::table>& toml_config)
{
    kinectfusion::GlobalConfiguration configuration;

    // cpptoml only supports int64_t, so we need to explicitly cast to int to suppress the warning
    auto volume_size_values = *toml_config->get_qualified_array_of<int64_t>("kinectfusion.volume_size");
    configuration.volume_size = make_int3(static_cast<int>(volume_size_values[0]),
                                          static_cast<int>(volume_size_values[1]),
                                          static_cast<int>(volume_size_values[2]));
    configuration.voxel_scale = static_cast<float>(*toml_config->get_qualified_as<double>("kinectfusion.voxel_scale"));
    configuration.bfilter_kernel_size = *toml_config->get_qualified_as<int>("kinectfusion.bfilter_kernel_size");
    configuration.bfilter_color_sigma  = static_cast<float>(*toml_config->get_qualified_as<double>("kinectfusion.bfilter_color_sigma"));
    configuration.bfilter_spatial_sigma  = static_cast<float>(*toml_config->get_qualified_as<double>("kinectfusion.bfilter_spatial_sigma"));
    configuration.init_depth  = static_cast<float>(*toml_config->get_qualified_as<double>("kinectfusion.init_depth"));
    configuration.use_output_frame = *toml_config->get_qualified_as<bool>("kinectfusion.use_output_frame");
    configuration.truncation_distance  = static_cast<float>(*toml_config->get_qualified_as<double>("kinectfusion.truncation_distance"));
    configuration.depth_cutoff_distance  = static_cast<float>(*toml_config->get_qualified_as<double>("kinectfusion.depth_cutoff_distance"));
    configuration.num_levels  = *toml_config->get_qualified_as<int>("kinectfusion.num_levels");
    configuration.triangles_buffer_size  = *toml_config->get_qualified_as<int>("kinectfusion.triangles_buffer_size");
    configuration.pointcloud_buffer_size  = *toml_config->get_qualified_as<int>("kinectfusion.pointcloud_buffer_size");
    configuration.distance_threshold  = static_cast<float>(*toml_config->get_qualified_as<double>("kinectfusion.distance_threshold"));
    configuration.angle_threshold  = static_cast<float>(*toml_config->get_qualified_as<double>("kinectfusion.angle_threshold"));
    auto icp_iterations_values = *toml_config->get_qualified_array_of<int64_t>("kinectfusion.icp_iterations");
    configuration.icp_iterations = {icp_iterations_values.begin(), icp_iterations_values.end()};

    return configuration;
}

auto make_camera(const std::shared_ptr<cpptoml::table>& toml_config)
{
    std::unique_ptr<DepthCamera> camera;

    const auto camera_type = *toml_config->get_qualified_as<std::string>("camera.type");
    if (camera_type == "Pseudo") {
        std::stringstream source_path {};
        source_path << data_path << "source/" << recording_name << "/";
        camera = std::make_unique<PseudoCamera>(source_path.str());
    } else if (camera_type == "Xtion") {
        camera = std::make_unique<XtionCamera>();
    } else if (camera_type == "RealSense") {
        if(*toml_config->get_qualified_as<bool>("camera.realsense.live")) {
            camera = std::make_unique<RealSenseCamera>();
        } else {
            std::stringstream source_file {};
            source_file << data_path << "source/" << recording_name << ".bag";
            camera = std::make_unique<RealSenseCamera>(source_file.str());
        }
    } else if (camera_type == "ZED") {
				camera = std::make_unique<ZEDMiniCamera>();
		} else {
        throw std::logic_error("There is no implementation for the camera type you specified.");
    }

    return camera;
}

void main_loop(const std::unique_ptr<DepthCamera> camera, const kinectfusion::GlobalConfiguration& configuration)
{
    kinectfusion::Pipeline pipeline { camera->get_parameters(), configuration };

    cv::namedWindow("Pipeline Output");
    for (bool end = false; !end;) {
        //1 Get frame
        InputFrame frame = camera->grab_frame();

        //2 Process frame
        bool success = pipeline.process_frame(frame.depth_map, frame.color_map);
        if (success)
            std::cout << "FRAME PROCESSED" << std::endl;
        if (!success)
            std::cout << "Frame could not be processed" << std::endl;

        //3 Display the output
        cv::imshow("Pipeline Output", pipeline.get_last_model_frame());

        switch (cv::waitKey(1)) {
            case 'a': { // Save all available data
                std::cout << "Saving all ..." << std::endl;
                std::cout << "Saving poses ..." << std::endl;
                auto poses = pipeline.get_poses();

                for (size_t i = 0; i < poses.size(); ++i) {
                    std::stringstream file_name {};
                    file_name << data_path << "poses/" << recording_name << "/seq_pose" << std::setfill('0')
                              << std::setw(5) << i << ".txt";
                    std::ofstream { file_name.str() } << poses[i] << std::endl;
                }

                std::cout << "Extracting mesh ..." << std::endl;
                auto mesh = pipeline.extract_mesh();
                std::cout << "Saving mesh ..." << std::endl;
                std::stringstream file_name {};
                file_name << data_path << "meshes/" << recording_name << ".ply";
                kinectfusion::export_ply(file_name.str(), mesh);
                end = true;
                break;
            }
            case 'p': { // Save poses only
                std::cout << "Saving poses ..." << std::endl;
                auto poses = pipeline.get_poses();

                for (size_t i = 0; i < poses.size(); ++i) {
                    std::stringstream file_name {};
                    file_name << data_path << "poses/" << recording_name << "/seq_pose" << std::setfill('0')
                              << std::setw(5) << i << ".txt";
                    std::ofstream { file_name.str() } << poses[i] << std::endl;
                }
                end = true;
                break;
            }
            case 'm': { // Save mesh only
                std::cout << "Extracting mesh ..." << std::endl;
                auto mesh = pipeline.extract_mesh();
                std::cout << "Saving mesh ..." << std::endl;
                std::stringstream file_name {};
                file_name << data_path << "meshes/" << recording_name << ".ply";
                kinectfusion::export_ply(file_name.str(), mesh);
                end = true;
                break;
            }
            case ' ': // Save nothing
                end = true;
                break;
            default:
                break;
        }
    }
}

void setup_cuda_device()
{
    auto n_devices = cv::cuda::getCudaEnabledDeviceCount();
    std::cout << "Found " << n_devices << " CUDA devices" << std::endl;
    for (int device_idx = 0; device_idx < n_devices; ++device_idx) {
        cv::cuda::DeviceInfo info { device_idx };
        std::cout << "Device #" << device_idx << ": " << info.name()
                  << " with " << info.totalMemory() / 1048576 << "MB total memory" << std::endl;
    }

    // Hardcoded to first device; change if necessary
    std::cout << "Using device #0" << std::endl;
    cv::cuda::setDevice(0);
}

class kinect_fusion : public threadloop {
public:
		kinect_fusion(std::string name_, phonebook *pb_)
		: threadloop{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		{
				rgb_depth_buffer = NULL;

				// Parse TOML configuration file
				auto toml_config = cpptoml::parse_file("../KinectFusionApp/KinectFusionApp/config.toml");
				data_path = *toml_config->get_as<std::string>("data_path");
				recording_name = *toml_config->get_as<std::string>("recording_name");

				setup_cuda_device();
				camera = make_camera(toml_config);
				configuration = make_configuration(toml_config);

				pipeline = new kinectfusion::Pipeline{ camera->get_parameters(), configuration };
		}

		virtual void start() override {
				threadloop::start();
				sb->schedule<rgb_depth_type>(id, "rgb_depth", [&](const rgb_depth_type *datum) {
						rgb_depth_buffer = datum;
				});
		}

		virtual void stop() override {
				threadloop::stop();
		}

		virtual ~kinect_fusion() override {
				delete pipeline;
		}

private:
		const std::shared_ptr<switchboard> sb;
		const rgb_depth_type* rgb_depth_buffer;

		kinectfusion::GlobalConfiguration configuration;
		std::unique_ptr<DepthCamera> camera;
		kinectfusion::Pipeline* pipeline;

protected:
		virtual skip_option _p_should_skip() override {
				return skip_option::run;
    }
		virtual void _p_thread_setup() override {
				cv::namedWindow("Pipeline Output");
		}

		virtual void _p_one_iteration() override {
				// ASUS Xtion
				// InputFrame frame = camera->grab_frame();

				// ZED
				if (!rgb_depth_buffer)
						return;

				cv::Mat depth{*rgb_depth_buffer->depth.value()};
				cv::Mat rgb{*rgb_depth_buffer->rgb.value()};
				cv::Mat rgb_convert;
				cv::cvtColor(rgb, rgb_convert, cv::COLOR_BGR2RGB);

				// cv::imshow("Pipeline Output", rgb);
				// cv::waitKey(1);

				InputFrame frame {depth, rgb_convert};
				// InputFrame frame {};
				// frame.depth_map = depth;
				// frame.color_map = rgb;

        //2 Process frame
        bool success = pipeline->process_frame(frame.depth_map, frame.color_map);
        if (success)
            std::cout << "FRAME PROCESSED" << std::endl;

        if (!success)
            std::cout << "Frame could not be processed" << std::endl;

        //3 Display the output
        cv::imshow("Pipeline Output", pipeline->get_last_model_frame());
				cv::waitKey(1);
		}
};

PLUGIN_MAIN(kinect_fusion);

int main(int argc, char* argv[]) { return 0; }

