/**
 * Acquire video and Game messages from a monitoring manager and displays them on screen.
 *
 * Depending on configuration of image providers, video streams and
 * meta_information are written
 */
#include <hl_communication/utils.h>
#include <hl_monitoring/monitoring_manager.h>
#include <hl_monitoring/utils.h>
#include <vive_provider/udp_message_manager.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Geometry>

#include <tclap/CmdLine.h>

using namespace hl_communication;
using namespace hl_monitoring;

int main(int argc, char** argv)
{
  TCLAP::CmdLine cmd("Acquire and display one or multiple streams along with meta-information", ' ', "0.9");

  TCLAP::ValueArg<std::string> config_arg("c", "config", "The path to the json configuration file", true, "config.json",
                                          "string", cmd);
  TCLAP::ValueArg<int> port_arg("p", "port", "The port on which vive messages are received (-1: no listener)", false,
                                -1, "port", cmd);
  TCLAP::ValueArg<std::string> log_arg("l", "log", "An optional path to existing vive logs", false, "", "path", cmd);
  TCLAP::ValueArg<int> time_arg("t", "time", "time_offset between start of the two streams (replay)", false, 0,
                                "time offset[us]", cmd);
  TCLAP::ValueArg<float> x_arg("", "x_center", "Value of field center along vive x_axis", false, 0, "x_center[m]", cmd);
  TCLAP::ValueArg<float> y_arg("", "y_center", "Value of field center along vive y_axis", false, 0, "x_center[m]", cmd);
  TCLAP::ValueArg<float> z_arg("", "z_center", "Value of field center along vive z_axis", false, 0, "z_center[m]", cmd);
  TCLAP::ValueArg<float> roll_arg("", "roll", "Order: roll-pitch-yaw, applied after translation", false, 0, "roll[deg]",
                                  cmd);
  TCLAP::ValueArg<float> pitch_arg("", "pitch", "Order: roll-pitch-yaw, applied after translation", false, 0,
                                   "pitch[deg]", cmd);
  TCLAP::ValueArg<float> yaw_arg("", "yaw", "Order: roll-pitch-yaw, applied after translation", false, 0, "yaw[deg]",
                                 cmd);

  try
  {
    cmd.parse(argc, argv);
  }
  catch (const TCLAP::ArgException& e)
  {
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
  }

  int port_read = port_arg.getValue();
  std::string log_path = log_arg.getValue();
  uint64_t time_offset = time_arg.getValue();
  Eigen::Vector3d center(x_arg.getValue(), y_arg.getValue(), z_arg.getValue());
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(yaw_arg.getValue() * M_PI / 180, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(pitch_arg.getValue() * M_PI / 180, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(roll_arg.getValue() * M_PI / 180, Eigen::Vector3d::UnitX());

  if (log_path == "" && port_read == -1)
  {
    std::cerr << "No log_path nor port_read provided, cannot acquire vive data" << std::endl;
    exit(EXIT_FAILURE);
  }

  MonitoringManager manager;
  manager.loadConfig(config_arg.getValue());

  vive_provider::UDPMessageManager vive_manager(port_arg.getValue(), -1);

  if (log_path != "")
  {
    std::cout << "loading messages" << std::endl;
    vive_manager.loadMessages(log_path);
    std::cout << "Nb messages loaded: " << vive_manager.getMessages().size() << std::endl;
    std::cout << "vive_manager start: " << vive_manager.getStart() << std::endl;
  }

  // While exit was not explicitly required, run
  uint64_t now = 0;
  uint64_t dt = 30 * 1000;  //[microseconds]
  int64_t vive_offset = 0;
  if (manager.isLive())
  {
    manager.setOffset(getSteadyClockOffset());
  }
  else
  {
    now = manager.getStart();
    std::cout << "monitoring_manager start: " << manager.getStart() << std::endl;
    int64_t default_offset = manager.getStart() - vive_manager.getStart();
    vive_offset = default_offset + time_offset;
    std::cout << "default_offset: " << default_offset << std::endl;
    std::cout << "vive_offset: " << vive_offset << std::endl;
  }
  while (manager.isGood())
  {
    manager.update();
    if (manager.isLive())
    {
      now = getTimeStamp() + manager.getOffset();
      ;
      vive_manager.autoUpdateOffset();
      vive_offset = vive_manager.getOffset();
    }
    else
    {
      now += dt;
    }
    int64_t vive_timestamp = now - vive_offset;
    std::cout << "vive_ts " << vive_timestamp << std::endl;
    vive_provider::GlobalMsg vive_msg = vive_manager.getMessage(vive_timestamp);

    for (const auto& entry : manager.getCalibratedImages(now))
    {
      cv::Mat display_img = entry.second.getImg().clone();
      if (entry.second.isFullySpecified())
      {
        const CameraMetaInformation& camera_information = entry.second.getCameraInformation();

        for (const vive_provider::TrackerMsg& tracker : vive_msg.trackers())
        {
          vive_provider::Vector3d pos = tracker.pos();
          Eigen::Vector3d tracker_in_vive(pos.x(), pos.y(), pos.z());
          Eigen::Vector3d tmp = tracker_in_vive - center;
          tmp = rotation * tmp;
          cv::Point3f tracker_in_field(tmp.x(), tmp.y(), tmp.z());
          cv::Point2f tracker_in_img = fieldToImg(tracker_in_field, camera_information);
          std::cout << "tracker in field" << tracker_in_field << std::endl;
          int circle_size = 4;
          cv::Scalar color(0, 0, 255);
          cv::circle(display_img, tracker_in_img, circle_size, color, cv::FILLED);
        }
        cv::imshow(entry.first, display_img);
      }
    }
    char key = cv::waitKey(1);
    if (key == 'q' || key == 'Q')
      break;
  }
}
