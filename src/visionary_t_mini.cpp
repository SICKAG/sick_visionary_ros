//
// Copyright (c) 2023 SICK AG, Waldkirch
//
// SPDX-License-Identifier: Unlicense

#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/ByteMultiArray.h>
#include <memory>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include "VisionaryControl.h"
#include "VisionaryDataStream.h"
#include "VisionaryTMiniData.h" // Header specific for the Time of Flight data

using namespace visionary;

std::shared_ptr<VisionaryControl> gControl;

std::shared_ptr<VisionaryTMiniData> gDataHandler;

image_transport::Publisher gPubDepth, gPubIntensity, gPubState;
ros::Publisher             gPubCameraInfo, gPubPoints;

diagnostic_updater::Updater updater;

std::string gFrameId;

boost::mutex gDataMtx;
bool         gReceive = true;

void driver_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "driver running");
    stat.add("frame_id", gFrameId);
    stat.add("device_ident", gControl->getDeviceIdent());
}

void publishCameraInfo(std_msgs::Header header, VisionaryTMiniData& dataHandler)
{
  sensor_msgs::CameraInfo ci;
  ci.header = header;

  ci.height = dataHandler.getHeight();
  ci.width  = dataHandler.getWidth();

  ci.D.clear();
  ci.D.resize(5, 0);
  ci.D[0] = dataHandler.getCameraParameters().k1;
  ci.D[1] = dataHandler.getCameraParameters().k2;
  ci.D[2] = dataHandler.getCameraParameters().p1;
  ci.D[3] = dataHandler.getCameraParameters().p2;
  ci.D[4] = dataHandler.getCameraParameters().k3;

  for (int i = 0; i < 9; i++)
  {
    ci.K[i] = 0;
  }
  ci.K[0] = dataHandler.getCameraParameters().fx;
  ci.K[4] = dataHandler.getCameraParameters().fy;
  ci.K[2] = dataHandler.getCameraParameters().cx;
  ci.K[5] = dataHandler.getCameraParameters().cy;
  ci.K[8] = 1;

  for (int i = 0; i < 12; i++)
    ci.P[i] = 0; // data.getCameraParameters().cam2worldMatrix[i];
  // TODO:....
  ci.P[0]  = dataHandler.getCameraParameters().fx;
  ci.P[5]  = dataHandler.getCameraParameters().fy;
  ci.P[10] = 1;
  ci.P[2]  = dataHandler.getCameraParameters().cx;
  ci.P[6]  = dataHandler.getCameraParameters().cy;

  gPubCameraInfo.publish(ci);
}

void publishDepth(std_msgs::Header header, VisionaryTMiniData& dataHandler)
{
  std::vector<uint16_t> vec = dataHandler.getDistanceMap();
  cv::Mat               m   = cv::Mat(dataHandler.getHeight(), dataHandler.getWidth(), CV_16UC1);
  memcpy(m.data, vec.data(), vec.size() * sizeof(uint16_t));
  sensor_msgs::ImagePtr msg =
    cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, m).toImageMsg();

  msg->header = header;
  gPubDepth.publish(msg);
}

void publishIntensity(std_msgs::Header header, VisionaryTMiniData& dataHandler)
{
  std::vector<uint16_t> vec = dataHandler.getIntensityMap();
  cv::Mat               m   = cv::Mat(dataHandler.getHeight(), dataHandler.getWidth(), CV_16UC1);
  memcpy(m.data, vec.data(), vec.size() * sizeof(uint16_t));
  sensor_msgs::ImagePtr msg =
    cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, m).toImageMsg();

  msg->header = header;
  gPubIntensity.publish(msg);
}

void publishStateMap(std_msgs::Header header, VisionaryTMiniData& dataHandler)
{
  std::vector<uint16_t> vec = dataHandler.getStateMap();
  cv::Mat               m   = cv::Mat(dataHandler.getHeight(), dataHandler.getWidth(), CV_16UC1);
  memcpy(m.data, vec.data(), vec.size() * sizeof(uint16_t));
  sensor_msgs::ImagePtr msg =
    cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, m).toImageMsg();

  msg->header = header;
  gPubState.publish(msg);
}

void publishPointCloud(std_msgs::Header header, VisionaryTMiniData& dataHandler)
{
  typedef sensor_msgs::PointCloud2 PointCloud;

  // Allocate new point cloud message
  PointCloud::Ptr cloudMsg(new PointCloud);
  cloudMsg->header       = header;
  cloudMsg->height       = dataHandler.getHeight();
  cloudMsg->width        = dataHandler.getWidth();
  cloudMsg->is_dense     = false;
  cloudMsg->is_bigendian = false;

  cloudMsg->fields.resize(5);
  cloudMsg->fields[0].name = "x";
  cloudMsg->fields[1].name = "y";
  cloudMsg->fields[2].name = "z";
  cloudMsg->fields[3].name = "intensity";
  int offset               = 0;
  for (size_t d = 0; d < 3; ++d, offset += sizeof(float))
  {
    cloudMsg->fields[d].offset   = offset;
    cloudMsg->fields[d].datatype = int(sensor_msgs::PointField::FLOAT32);
    cloudMsg->fields[d].count    = 1;
  }

  cloudMsg->fields[3].offset   = offset;
  cloudMsg->fields[3].datatype = int(sensor_msgs::PointField::UINT16);
  cloudMsg->fields[3].count    = 1;
  offset += sizeof(uint16_t);

  cloudMsg->point_step = offset;
  cloudMsg->row_step   = cloudMsg->point_step * cloudMsg->width;
  cloudMsg->data.resize(cloudMsg->height * cloudMsg->row_step);

  std::vector<PointXYZ> pointCloud;
  dataHandler.generatePointCloud(pointCloud);
  dataHandler.transformPointCloud(pointCloud);

  // simple copy to create a XYZ point cloud
  // memcpy(&cloud_msg->data[0], &pointCloud[0], pointCloud.size()*sizeof(PointXYZ));

  std::vector<uint16_t>::const_iterator itIntens  = dataHandler.getIntensityMap().begin();
  std::vector<PointXYZ>::const_iterator itPC      = pointCloud.begin();
  size_t                                cloudSize = dataHandler.getHeight() * dataHandler.getWidth();
  for (size_t index = 0; index < cloudSize; ++index, ++itIntens, ++itPC)
  {
    memcpy(&cloudMsg->data[index * cloudMsg->point_step + cloudMsg->fields[0].offset], &*itPC, sizeof(PointXYZ));
    memcpy(&cloudMsg->data[index * cloudMsg->point_step + cloudMsg->fields[3].offset], &*itIntens, sizeof(uint16_t));
  }
  gPubPoints.publish(cloudMsg);
}

void publish_frame(VisionaryTMiniData& dataHandler)
{
  bool publishedAnything = false;

  std_msgs::Header header;
  header.stamp    = ros::Time::now();
  header.frame_id = gFrameId;

  if (gPubCameraInfo.getNumSubscribers() > 0)
  {
    publishedAnything = true;
    publishCameraInfo(header, dataHandler);
  }
  if (gPubDepth.getNumSubscribers() > 0)
  {
    publishedAnything = true;
    publishDepth(header, dataHandler);
  }
  if (gPubIntensity.getNumSubscribers() > 0)
  {
    publishedAnything = true;
    publishIntensity(header, dataHandler);
  }
  if (gPubState.getNumSubscribers() > 0)
  {
    publishedAnything = true;
    publishStateMap(header, dataHandler);
  }
  if (gPubPoints.getNumSubscribers() > 0)
  {
    publishedAnything = true;
    publishPointCloud(header, dataHandler);
  }

  if (!publishedAnything)
  {
    ROS_DEBUG("Nothing published");
    if (gControl)
      gControl->stopAcquisition();
  }
}

void thr_publish_frame()
{
  gDataMtx.lock();
  publish_frame(*gDataHandler);
  gDataMtx.unlock();
}

void thr_receive_frame(std::shared_ptr<VisionaryDataStream> pDataStream,
                       std::shared_ptr<VisionaryTMiniData>  pDataHandler)
{
  while (gReceive)
  {
    if (!pDataStream->getNextFrame())
    {
      continue; // No valid frame received
    }
    if (gDataMtx.try_lock())
    {
      gDataHandler = pDataHandler;
      gDataMtx.unlock();
      boost::thread thr(&thr_publish_frame);
    }
    else
      ROS_INFO("skipping frame with number %d", pDataHandler->getFrameNum());
  }
}

void _on_new_subscriber()
{
  ROS_DEBUG("Got new subscriber");
  if (gControl)
    gControl->startAcquisition();
}

void on_new_subscriber_ros(const ros::SingleSubscriberPublisher&)
{
  _on_new_subscriber();
}

void on_new_subscriber_it(const image_transport::SingleSubscriberPublisher&)
{
  _on_new_subscriber();
}

int main(int argc, char** argv)
{
  //ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  ros::init(argc, argv, "sick_visionary_t_mini");
  ros::NodeHandle nh("~");

  // default parameters
  std::string remoteDeviceIp = "192.168.1.10";
  gFrameId                   = "camera";

  ros::param::get("~remote_device_ip", remoteDeviceIp);
  ros::param::get("~frame_id", gFrameId);

  std::shared_ptr<VisionaryTMiniData>  pDataHandler = std::make_shared<VisionaryTMiniData>();
  std::shared_ptr<VisionaryDataStream> pDataStream  = std::make_shared<VisionaryDataStream>(pDataHandler);
  gControl                                          = std::make_shared<VisionaryControl>();

  ROS_INFO("Connecting to device at %s", remoteDeviceIp.c_str());
  if (!gControl->open(VisionaryControl::ProtocolType::COLA_2, remoteDeviceIp.c_str(), 5000 /*ms*/))
  {
    ROS_ERROR("Connection with devices control channel failed");
    return -1;
  }
  // To be sure the acquisition is currently stopped.
  gControl->stopAcquisition();

  if (!pDataStream->open(remoteDeviceIp.c_str(), 2114u))
  {
    ROS_ERROR("Connection with devices data channel failed");
    return -1;
  }

  // TODO: add get device name and device version and print to ros info.
  ROS_INFO("Connected with Visionary-T Mini");

  // make me public (after init.)
  image_transport::ImageTransport it(nh);
  gPubCameraInfo = nh.advertise<sensor_msgs::CameraInfo>(
    "camera_info", 1, (ros::SubscriberStatusCallback)on_new_subscriber_ros, ros::SubscriberStatusCallback());
  gPubDepth  = it.advertise("depth",
                           1,
                           (image_transport::SubscriberStatusCallback)on_new_subscriber_it,
                           image_transport::SubscriberStatusCallback());
  gPubPoints = nh.advertise<sensor_msgs::PointCloud2>(
    "points", 2, (ros::SubscriberStatusCallback)on_new_subscriber_ros, ros::SubscriberStatusCallback());
  gPubIntensity = it.advertise("intensity",
                               1,
                               (image_transport::SubscriberStatusCallback)on_new_subscriber_it,
                               image_transport::SubscriberStatusCallback());
  gPubState     = it.advertise("statemap",
                           1,
                           (image_transport::SubscriberStatusCallback)on_new_subscriber_it,
                           image_transport::SubscriberStatusCallback());

  //diagnostics
  updater.setHardwareID(nh.getNamespace());
  updater.add("driver", driver_diagnostics);

  // start receiver thread for camera images
  boost::thread rec_thr(boost::bind(&thr_receive_frame, pDataStream, pDataHandler));

  // wait til end of exec.
  ros::spin();

  gReceive = false;
  rec_thr.join();

  gControl->startAcquisition();
  gControl->close();
  pDataStream->close();

  gPubDepth.shutdown();
  gPubPoints.shutdown();
  gPubIntensity.shutdown();
  gPubState.shutdown();
  gPubCameraInfo.shutdown();

  return 0;
}
