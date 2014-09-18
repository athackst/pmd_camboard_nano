/******************************************************************************
 * Copyright (c) 2012 Sergey Alexandrov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ******************************************************************************/

#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <boost/thread.hpp>

#include <pmd_camboard_nano/PMDConfig.h>
#include "pmd_camboard_nano.h"
#include "pmd_exceptions.h"

namespace pmd_camboard_nano
{

class DriverNodelet : public nodelet::Nodelet
{

public:

  virtual ~DriverNodelet()
  {
    // Make sure we interrupt initialization (if it happened to still execute).
    init_thread_.interrupt();
    init_thread_.join();
  }

private:
  virtual void onInit()
  {
        // We will be retrying to open camera until it is open, which may block the
        // thread. Nodelet::onInit() should not block, hence spawning a new thread
        // to do initialization.
        init_thread_ = boost::thread(boost::bind(&DriverNodelet::onInitImpl, this));
  }

  void onInitImpl()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& pn = getPrivateNodeHandle();


    // Retrieve parameters from server
    update_rate=30;
    pn.param<std::string>("frame_id", frame_id_, "/camera_optical_frame");
    NODELET_INFO("Loaded param frame_id: %s", frame_id_.c_str());
    pn.param<std::string>("device_serial", device_serial, "");
    NODELET_INFO("Loaded param device_serial: %s", device_serial.c_str());
    pn.param<double>("open_camera_retry_period", open_camera_retry_period, 3.);
    pn.param<std::string>("plugin_dir", plugin_dir, "/usr/local/pmd/plugins");
    NODELET_INFO("Loaded param plugin_dir: %s", plugin_dir.c_str());
    pn.param<std::string>("source_plugin", source_plugin, "camboardnano");
    NODELET_INFO("Loaded param source_plugin: %s", source_plugin.c_str());
    pn.param<std::string>("process_plugin", process_plugin, "camboardnanoproc");
    NODELET_INFO("Loaded param process_plugin: %s", process_plugin.c_str());

    // Setup updater
    camera_state_ = OPENING;
    state_info_ = "opening camera " + device_serial;
    updater.setHardwareIDf("%s", device_serial.c_str());
    if(device_serial.empty())
    {
        updater.setHardwareID("unknown");
    }
    updater.add(getName().c_str(), this, &DriverNodelet::getCurrentState);

    // Setup periodic callback to get new data from the camera
    update_timer_ = nh.createTimer(ros::Rate(update_rate).expectedCycleTime(), &DriverNodelet::updateCallback, this, false ,false);

    // Open camera
    openCamera(update_timer_);

    // Advertise topics
    ros::NodeHandle distance_nh(nh, "distance");
    image_transport::ImageTransport distance_it(distance_nh);
    distance_publisher_ = distance_it.advertiseCamera("image", 1);
    ros::NodeHandle depth_nh(nh, "depth");
    image_transport::ImageTransport depth_it(depth_nh);
    depth_publisher_ = depth_it.advertiseCamera("image", 1);
    ros::NodeHandle amplitude_nh(nh, "amplitude");
    image_transport::ImageTransport amplitude_it(amplitude_nh);
    amplitude_publisher_ = amplitude_it.advertiseCamera("image", 1);
    points_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("points_unrectified", 1);

    // Setup dynamic reconfigure server
    reconfigure_server_.reset(new ReconfigureServer(config_mutex_, pn));
    ReconfigureServer::CallbackType f = boost::bind(&DriverNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);
  }

  void openCamera(ros::Timer &timer)
  {
      timer.stop();
      while (!camera_)
      {
        try
        {
          boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
          camera_ = boost::make_shared<PMDCamboardNano>(device_serial, plugin_dir, source_plugin, process_plugin);
          device_serial = camera_->getSerialNumber().c_str();
          updater.setHardwareIDf("%s", device_serial.c_str());
          NODELET_INFO("Opened PMD camera with serial number \"%s\"", camera_->getSerialNumber().c_str());
          loadCalibrationData();
          NODELET_INFO("Loaded calibration data");
          camera_->update();

          camera_info_ = camera_->getCameraInfo();
        }
        catch (PMDCameraNotOpenedException& e)
        {
          camera_state_ = CAMERA_NOT_FOUND;
          if (device_serial != "")
          {
            std::stringstream err;
            err << "Unable to open PMD camera with serial number " << device_serial;
            state_info_ = err.str();
            NODELET_INFO("%s",state_info_.c_str());
          }
          else
          {
              state_info_ = "Unable to open PMD camera..";
              NODELET_INFO("%s",state_info_.c_str());
          }
          boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
          camera_.reset();
        }
        updater.update();
        boost::this_thread::sleep(boost::posix_time::seconds(open_camera_retry_period));
      }
      timer.start();
  }

  void updateCallback(const ros::TimerEvent& event)
  {
    // Download the most recent data from the device 
    camera_state_ = OK;
    state_info_ = "Camera operating nominally";
    try
    {
        boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
        camera_->update();
    }
    catch(std::exception &e)
    {
        //we have disconnected from the camera, try to reinitialize
        camera_state_ = ERROR;
        state_info_ = "Unable to read from camera!";
        NODELET_ERROR("%s", state_info_.c_str());
        boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
        camera_.reset();
        init_thread_ = boost::thread(boost::bind(&DriverNodelet::openCamera, this, _1), update_timer_);
        return;
    }
    camera_info_->header.frame_id = frame_id_;
    // Get new data and publish for the topics that have subscribers
    // Distance
    if (distance_publisher_.getNumSubscribers() > 0)
    {
        boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
        sensor_msgs::ImagePtr distance = camera_->getDistanceImage();
        distance->header.frame_id = frame_id_;
        camera_info_->header.stamp = distance->header.stamp;
        distance_publisher_.publish(distance, camera_info_);
    }
    // Depth
    if (depth_publisher_.getNumSubscribers() > 0)
    {
        boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
        sensor_msgs::ImagePtr depth = camera_->getDepthImage();
        depth->header.frame_id = frame_id_;
        camera_info_->header.stamp = depth->header.stamp;
        depth_publisher_.publish(depth, camera_info_);
    }
    // Amplitude
    if (amplitude_publisher_.getNumSubscribers() > 0)
    {
        boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
        sensor_msgs::ImagePtr amplitude = camera_->getAmplitudeImage();
        amplitude->header.frame_id = frame_id_;
        camera_info_->header.stamp = amplitude->header.stamp;
        amplitude_publisher_.publish(amplitude, camera_info_);
    }
    // Points
    if (points_publisher_.getNumSubscribers() > 0)
    {
        boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
        sensor_msgs::PointCloud2Ptr points = camera_->getPointCloud();
        points->header.frame_id = frame_id_;
        points_publisher_.publish(points);
    }
    updater.update();
  }

  void reconfigureCallback(pmd_camboard_nano::PMDConfig &config, uint32_t level)
  {
      if(config.update_rate != update_rate)
      {
          update_rate = config.update_rate;
          update_timer_.stop();
          update_timer_.setPeriod(ros::Rate(update_rate).expectedCycleTime());
          update_timer_.start();
      }
      camera_->setFlipVertical(config.flip_vertical);
      camera_->setRemoveInvalidPixels(config.remove_invalid_pixels);
      config.integration_time = camera_->setIntegrationTime(config.integration_time);
      camera_->setAveragingFrames(config.averaging_frames);
      camera_->setSignalStrengthCheck(config.signal_strength_check);
      camera_->setSignalStrengthThreshold(config.signal_strength_threshold);
      camera_->setConsistencyCheck(config.consistency_check);
      camera_->setConsistencyThreshold(config.consistency_threshold);
      camera_->setBilateralFilter(config.bilateral_filter);
      camera_->setBilateralFilterSigmaSpatial(config.sigma_spatial);
      camera_->setBilateralFilterSigmaRange(config.sigma_range);
      camera_->setBilateralFilterKernelSize(config.kernel_size);
      camera_->setBilateralFilterEnhanceImage(config.bilateral_filter_enhance_image);
      config_ = config;
  }

private:

  void loadCalibrationData()
  {
    ros::NodeHandle& pn = getPrivateNodeHandle();
    std::string calibration_file;
    // Try to load a specific calibration file (if requested)
    if (pn.getParam("calibration_file", calibration_file) && !calibration_file.empty())
    {
      NODELET_INFO("Trying to load calibration from \"%s\"", calibration_file.c_str());
      try
      {
          boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
          camera_->loadCalibrationData(calibration_file);
          NODELET_INFO("Loaded calibration data from \"%s\"", calibration_file.c_str());
          return;
      }
      catch(PMDCameraNotOpenedException& e)
      {
          NODELET_WARN("Failed to load calibration data from \"%s\"", calibration_file.c_str());
      }
    }
    // Check whether the calibration data was loaded from the default location
    if (camera_->isCalibrationDataLoaded())
    {
      NODELET_INFO("Loaded calibration data from default location (\"%s.dat\" in the working directory)", camera_->getSerialNumber().c_str());
    }
    else
    {
      NODELET_WARN("Will use default calibration data");
    }
  }

  void getCurrentState(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
      stat.add("Serial", device_serial);
      switch (camera_state_)
      {
          case OPENING:
              stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, state_info_);
              break;
          case CAMERA_NOT_FOUND:
              stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, state_info_ );
              break;
          case OK:
              stat.summary(diagnostic_msgs::DiagnosticStatus::OK, state_info_);
              break;
          case ERROR:
              stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, state_info_);
              break;
          default:
              break;
      }
  }

  PMDCamboardNano::Ptr camera_;
  boost::thread init_thread_;
  ros::Timer update_timer_;

  image_transport::CameraPublisher distance_publisher_;
  image_transport::CameraPublisher depth_publisher_;
  image_transport::CameraPublisher amplitude_publisher_;
  ros::Publisher points_publisher_;
  std::string frame_id_;
  boost::recursive_mutex config_mutex_;
  typedef dynamic_reconfigure::Server<pmd_camboard_nano::PMDConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  pmd_camboard_nano::PMDConfig config_;
  //sensor_msgs::CameraInfoPtr camera_info_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager>  camera_info_mgr_;

  std::string device_serial, plugin_dir, source_plugin, process_plugin;
  double open_camera_retry_period;

  // Dynamic reconfigure params
  double update_rate;
  bool   flip_vertical;
  bool   remove_invalid_pixels;
  int    integration_time;
  int    averaging_frames;
  bool   signal_strength_check;
  int    signal_strength_threshold;
  bool   consistency_check;
  double consistency_threshold;
  bool   bilateral_filter;
  double sigma_spatial;
  double sigma_range;
  int    kernel_size;
  bool   bilateral_filter_enhance_image;

  // State updater params
  enum CameraState
  {
      OPENING,
      CAMERA_NOT_FOUND,
      ERROR,
      OK
  }camera_state_;
  std::string state_info_;

  diagnostic_updater::Updater updater;

};

}

// Register as a nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (pmd_camboard_nano, driver, pmd_camboard_nano::DriverNodelet, nodelet::Nodelet);

