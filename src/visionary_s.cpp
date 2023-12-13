//
// Copyright (c) 2023 SICK AG, Waldkirch
//
// SPDX-License-Identifier: Unlicense

#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/ByteMultiArray.h>
#include <memory>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include "VisionaryControl.h"
#include "VisionaryDataStream.h"
#include "VisionarySData.h" // Header specific for the Stereo data

using namespace visionary;

std::shared_ptr<VisionaryControl> gControl;

std::shared_ptr<VisionarySData> gDataHandler;

image_transport::Publisher gPubZ, gPubStatemap, gPubRGBA;
ros::Publisher             gPubCameraInfo, gPubPoints /*, gPubIos*/;

std::shared_ptr<diagnostic_updater::Updater>         updater;
std::shared_ptr<diagnostic_updater::TopicDiagnostic> gPubZ_freq, gPubStatemap_freq, gPubRGBA_freq;
std::shared_ptr<diagnostic_updater::TopicDiagnostic> gPubCameraInfo_freq, gPubPoints_freq;

std::string gFrameId;
std::string gDeviceIdent;
bool        gEnableZ, gEnableStatemap, gEnableRGBA, gEnablePoints;

boost::mutex gDataMtx;
bool         gReceive = true;

int gNumSubs = 0;

void diag_timer_cb(const ros::TimerEvent&)
{
  updater->update();
}

void driver_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "driver running");
  stat.add("frame_id", gFrameId);
  stat.add("device_ident", gDeviceIdent);
  stat.add("NumSubscribers_CameraInfo", gPubCameraInfo.getNumSubscribers());
  stat.add("gEnablePoints", gEnablePoints);
  if (gEnablePoints)
    stat.add("NumSubscribers_Points", gPubPoints.getNumSubscribers());
  stat.add("gEnableZ", gEnableZ);
  if (gEnableZ)
    stat.add("NumSubscribers_Z", gPubZ.getNumSubscribers());
  stat.add("gEnableStatemap", gEnableStatemap);
  if (gEnableStatemap)
    stat.add("NumSubscribers_Statemap", gPubStatemap.getNumSubscribers());
  stat.add("gEnableRGBA", gEnableRGBA);
  if (gEnableRGBA)
    stat.add("NumSubscribers_RGBA", gPubRGBA.getNumSubscribers());
}

void publishCameraInfo(std_msgs::Header header, VisionarySData& dataHandler)
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

void publishZ(std_msgs::Header header, VisionarySData& dataHandler)
{
  std::vector<uint16_t> vec = dataHandler.getZMap();
  cv::Mat               m   = cv::Mat(dataHandler.getHeight(), dataHandler.getWidth(), CV_16UC1);
  memcpy(m.data, vec.data(), vec.size() * sizeof(uint16_t));
  sensor_msgs::ImagePtr msg =
    cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, m).toImageMsg();

  msg->header = header;
  gPubZ.publish(msg);
}

void publishStatemap(std_msgs::Header header, VisionarySData& dataHandler)
{
  std::vector<uint16_t> vec = dataHandler.getConfidenceMap();
  cv::Mat               m   = cv::Mat(dataHandler.getHeight(), dataHandler.getWidth(), CV_16UC1);
  memcpy(m.data, vec.data(), vec.size() * sizeof(uint16_t));
  sensor_msgs::ImagePtr msg =
    cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, m).toImageMsg();

  msg->header = header;
  gPubStatemap.publish(msg);
}

void publishRGBA(std_msgs::Header header, VisionarySData& dataHandler)
{
  std::vector<uint32_t> vec = dataHandler.getRGBAMap();
  cv::Mat               m   = cv::Mat(dataHandler.getHeight(), dataHandler.getWidth(), CV_8UC4);
  memcpy(m.data, vec.data(), vec.size() * sizeof(uint32_t));
  sensor_msgs::ImagePtr msg =
    cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGBA8, m).toImageMsg();

  msg->header = header;
  gPubRGBA.publish(msg);
}

void publishPointCloud(std_msgs::Header header, VisionarySData& dataHandler)
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
  cloudMsg->fields[3].name = "confidence";
  cloudMsg->fields[4].name = "rgba";
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

  cloudMsg->fields[4].offset   = offset;
  cloudMsg->fields[4].datatype = int(sensor_msgs::PointField::UINT32);
  cloudMsg->fields[4].count    = 1;
  offset += sizeof(uint32_t);

  cloudMsg->point_step = offset;
  cloudMsg->row_step   = cloudMsg->point_step * cloudMsg->width;
  cloudMsg->data.resize(cloudMsg->height * cloudMsg->row_step);

  std::vector<PointXYZ> pointCloud;
  dataHandler.generatePointCloud(pointCloud);
  dataHandler.transformPointCloud(pointCloud);

  // simple copy to create a XYZ point cloud
  // memcpy(&cloud_msg->data[0], &pointCloud[0], pointCloud.size()*sizeof(PointXYZ));

  std::vector<uint16_t>::const_iterator itConf    = dataHandler.getConfidenceMap().begin();
  std::vector<uint32_t>::const_iterator itRGBA    = dataHandler.getRGBAMap().begin();
  std::vector<PointXYZ>::const_iterator itPC      = pointCloud.begin();
  size_t                                cloudSize = dataHandler.getHeight() * dataHandler.getWidth();
  for (size_t index = 0; index < cloudSize; ++index, ++itConf, ++itRGBA, ++itPC)
  {
    memcpy(&cloudMsg->data[index * cloudMsg->point_step + cloudMsg->fields[0].offset], &*itPC, sizeof(PointXYZ));

    memcpy(&cloudMsg->data[index * cloudMsg->point_step + cloudMsg->fields[3].offset], &*itConf, sizeof(uint16_t));
    memcpy(&cloudMsg->data[index * cloudMsg->point_step + cloudMsg->fields[4].offset], &*itRGBA, sizeof(uint32_t));
  }
  gPubPoints.publish(cloudMsg);
}

void publish_frame(VisionarySData& dataHandler)
{
  bool publishedAnything = false;

  std_msgs::Header header;
  header.stamp    = ros::Time::now();
  header.frame_id = gFrameId;

  if (gPubCameraInfo.getNumSubscribers() > 0)
  {
    publishedAnything = true;
    publishCameraInfo(header, dataHandler);
    // gPubCameraInfo_freq->tick(header.stamp);
  }
  if (gEnableZ && gPubZ.getNumSubscribers() > 0)
  {
    publishedAnything = true;
    publishZ(header, dataHandler);
    // gPubZ_freq->tick(header.stamp);
  }
  if (gEnableStatemap && gPubStatemap.getNumSubscribers() > 0)
  {
    publishedAnything = true;
    publishStatemap(header, dataHandler);
    // gPubStatemap_freq->tick(header.stamp);
  }
  if (gEnableRGBA && gPubRGBA.getNumSubscribers() > 0)
  {
    publishedAnything = true;
    publishRGBA(header, dataHandler);
    // gPubRGBA_freq->tick(header.stamp);
  }
  if (gEnablePoints && gPubPoints.getNumSubscribers() > 0)
  {
    publishedAnything = true;
    publishPointCloud(header, dataHandler);
    // gPubPoints_freq->tick(header.stamp);
  }

  if (publishedAnything)
  {
    gPubCameraInfo_freq->tick(header.stamp);
    if (gEnableZ)
      gPubZ_freq->tick(header.stamp);
    if (gEnableStatemap)
      gPubStatemap_freq->tick(header.stamp);
    if (gEnableRGBA)
      gPubRGBA_freq->tick(header.stamp);
    if (gEnablePoints)
      gPubPoints_freq->tick(header.stamp);
  }
  else
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

void thr_receive_frame(std::shared_ptr<VisionaryDataStream> pDataStream, std::shared_ptr<VisionarySData> pDataHandler)
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
  gNumSubs++;
  ROS_DEBUG_STREAM("Got new subscriber, total amount of subscribers: " << gNumSubs);
  if (gControl)
    gControl->startAcquisition();
}

void _on_subscriber_disconnected()
{
  gNumSubs--;
  ROS_DEBUG_STREAM("Subscriber disconnected, total amount of subscribers: " << gNumSubs);
}

void on_new_subscriber_ros(const ros::SingleSubscriberPublisher&)
{
  _on_new_subscriber();
}

void on_new_subscriber_it(const image_transport::SingleSubscriberPublisher&)
{
  _on_new_subscriber();
}

void on_subscriber_disconnected_ros(const ros::SingleSubscriberPublisher&)
{
  _on_subscriber_disconnected();
}

void on_subscriber_disconnected_it(const image_transport::SingleSubscriberPublisher&)
{
  _on_subscriber_disconnected();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sick_visionary_s");
  ros::NodeHandle nh("~");

  // default parameters
  std::string remoteDeviceIp = "192.168.1.10";
  gFrameId                   = "camera";

  ros::param::get("~remote_device_ip", remoteDeviceIp);
  ros::param::get("~frame_id", gFrameId);
  ros::param::get("~enable_z", gEnableZ);
  ros::param::get("~enable_statemap", gEnableStatemap);
  ros::param::get("~enable_rgba", gEnableRGBA);
  ros::param::get("~enable_points", gEnablePoints);

  std::shared_ptr<VisionarySData>      pDataHandler = std::make_shared<VisionarySData>();
  std::shared_ptr<VisionaryDataStream> pDataStream  = std::make_shared<VisionaryDataStream>(pDataHandler);
  gControl                                          = std::make_shared<VisionaryControl>();

  ROS_INFO("Connecting to device at %s", remoteDeviceIp.c_str());
  if (!gControl->open(VisionaryControl::ProtocolType::COLA_B, remoteDeviceIp.c_str(), 5000 /*ms*/))
  {
    ROS_ERROR("Connection with devices control channel failed");
    return -1;
  }
  // To be sure the acquisition is currently stopped.
  gControl->stopAcquisition();

  if (!pDataStream->open(remoteDeviceIp.c_str(), 2114))
  {
    ROS_ERROR("Connection with devices data channel failed");
    return -1;
  }

  // TODO: add get device name and device version and print to ros info.
  ROS_INFO("Connected with Visionary-S");

  // make me public (after init.)
  image_transport::ImageTransport it(nh);
  gPubCameraInfo = nh.advertise<sensor_msgs::CameraInfo>("camera_info",
                                                         1,
                                                         (ros::SubscriberStatusCallback)on_new_subscriber_ros,
                                                         (ros::SubscriberStatusCallback)on_subscriber_disconnected_ros);
  if (gEnablePoints)
    gPubPoints = nh.advertise<sensor_msgs::PointCloud2>("points",
                                                        2,
                                                        (ros::SubscriberStatusCallback)on_new_subscriber_ros,
                                                        (ros::SubscriberStatusCallback)on_subscriber_disconnected_ros);
  if (gEnableZ)
    gPubZ = it.advertise("z",
                         1,
                         (image_transport::SubscriberStatusCallback)on_new_subscriber_it,
                         (image_transport::SubscriberStatusCallback)on_subscriber_disconnected_it);
  if (gEnableStatemap)
    gPubStatemap = it.advertise("statemap",
                                1,
                                (image_transport::SubscriberStatusCallback)on_new_subscriber_it,
                                (image_transport::SubscriberStatusCallback)on_subscriber_disconnected_it);
  if (gEnableRGBA)
    gPubRGBA = it.advertise("rgba",
                            1,
                            (image_transport::SubscriberStatusCallback)on_new_subscriber_it,
                            (image_transport::SubscriberStatusCallback)on_subscriber_disconnected_it);

  gDeviceIdent = gControl->getDeviceIdent();

  // diagnostics
  updater.reset(new diagnostic_updater::Updater());
  updater->setHardwareID(nh.getNamespace());
  updater->add("driver", driver_diagnostics);

  double desiredFreq; // device max freq is 30FPS
  ros::param::get("~desired_frequency", desiredFreq);
  double min_freq = desiredFreq * 0.9;
  double max_freq = desiredFreq * 1.1;
  gPubCameraInfo_freq.reset(
    new diagnostic_updater::TopicDiagnostic("camera_info",
                                            *updater,
                                            diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq),
                                            diagnostic_updater::TimeStampStatusParam()));
  if (gEnablePoints)
    gPubPoints_freq.reset(
      new diagnostic_updater::TopicDiagnostic("points",
                                              *updater,
                                              diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq),
                                              diagnostic_updater::TimeStampStatusParam()));
  if (gEnableZ)
    gPubZ_freq.reset(
      new diagnostic_updater::TopicDiagnostic("z",
                                              *updater,
                                              diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq),
                                              diagnostic_updater::TimeStampStatusParam()));
  if (gEnableStatemap)
    gPubStatemap_freq.reset(
      new diagnostic_updater::TopicDiagnostic("statemap",
                                              *updater,
                                              diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq),
                                              diagnostic_updater::TimeStampStatusParam()));
  if (gEnableRGBA)
    gPubRGBA_freq.reset(
      new diagnostic_updater::TopicDiagnostic("rgba",
                                              *updater,
                                              diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq),
                                              diagnostic_updater::TimeStampStatusParam()));

  ros::Timer timer = nh.createTimer(ros::Duration(1.0), diag_timer_cb);

  // start receiver thread for camera images
  boost::thread rec_thr(boost::bind(&thr_receive_frame, pDataStream, pDataHandler));

  // wait til end of exec.
  ros::spin();

  gReceive = false;
  rec_thr.join();

  gControl->stopAcquisition();
  gControl->close();
  pDataStream->close();

  if (gEnableZ)
    gPubZ.shutdown();
  if (gEnableStatemap)
    gPubStatemap.shutdown();
  if (gEnableRGBA)
    gPubRGBA.shutdown();
  if (gEnablePoints)
    gPubPoints.shutdown();
  gPubCameraInfo.shutdown();
  // gPubIos.shutdown();

  return 0;
}
