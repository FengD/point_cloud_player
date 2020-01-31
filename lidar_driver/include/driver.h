/**
  * Copyright (C) 2019 Hirain Technologies
  * License: Modified BSD Software License Agreement
  * Author: Feng DING, Xu WANG
  * Description: The driver which used to receive the udp packets and process the packets to point cloud
  * Revise 1.0:
  *     201908161642 add cut angle mode
  *     By Wang Xu
  */
#ifndef _DRIVER_H_
#define _DRIVER_H_

#include <pthread.h>
#include <semaphore.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <list>
#include <string>
#include <vector>

#define UDPPACKETSIZE (14000)

namespace itd_lidar {

namespace lidar_driver {
class InputBase;

class RawData;

class RawDataFactory;

// the udp packet struct
struct Packet{
  uint8_t data[UDPPACKETSIZE];
};

class Driver {
 public:
  /**
  * constructor
  * @param {string} source ip address
  * @param {string} multicast ip address, could be set as empty if you do not need
  * @param {int} receive data port
  * @param {string} model of the lidar "like P40P, VLP16, etc."
  * @param {int} lidar mode {single return (0), dual return (1)}
  * @param {string} the path of the correction file, if it is empty the default value will be used
  * @param {float} cut angle, below zero means deactivated cut angle mode
  * @param {methode} callback
  */
  Driver(const std::string& source_ip,
         const std::string& multicast_ip,
         const int& data_port,
         const std::string& model,
         const int& mode,
         const std::string& correction_file,
         float cut_angle,
         boost::function<void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, int timestamp)> lidar_callback);

  /**
  * constructor
  * @param {string} source ip address
  * @param {string} multicast ip address, could be set as empty if you do not need
  * @param {int} receive data port
  * @param {string} model of the lidar "like P40P, VLP16, etc."
  * @param {int} lidar mode {single return (0), dual return (1)}
  * @param {string} the path of the correction file, if it is empty the default value will be used
  * @param {methode} callback
  */
  Driver(const std::string& source_ip,
         const std::string& multicast_ip,
         const int& data_port,
         const std::string& model,
         const int& mode,
         const std::string& correction_file,
         boost::function<void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, int timestamp)> lidar_callback);

  /**
  * constructor
  * @param {string} source ip address
  * @param {string} multicast ip address, could be set as empty if you do not need
  * @param {int} receive data port
  * @param {string} model of the lidar "like P40P, VLP16, etc."
  * @param {int} lidar mode {single return (0), dual return (1)}
  * @param {string} vector containing all correction files: angle.csv ChannelNum.csv CurveRate.csv curves.csv
  * @param {float} cut angle, below zero means deactivated cut angle mode
  * @param {methode} callback
  */
  Driver(const std::string& source_ip,
         const std::string& multicast_ip,
         const int& data_port,
         const std::string& model,
         const int& mode,
         const int& direction,
         const int& version,
         std::vector<std::string> correctionfileList,
         float cut_angle,
         boost::function<void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, int timestamp)> lidar_callback,
         const std::string& pcapFilePath = "");

  Driver(const std::string& source_ip,
         const std::string& multicast_ip,
         const int& data_port,
         const std::string& model,
         const int& mode,
         const int& direction,
         const int& version,
         std::vector<std::string> correctionfileList,
         boost::function<void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, int timestamp)> lidar_callback);

  /**
  * destructor
  */
  virtual ~Driver();

  /**
  * @brief {start function used to start the threads
  */
  void Start();

  /**
  * @brief {stop function used to stop the threads}
  */
  void Stop();

  void SetLidarCallBack(boost::function<void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, int timestamp)> lidar_callback);

 private:
  /*
   * @brief the function of the lidar_recv_thread_
   */
  void LidarPacketRecvFn();
  /*
   * @brief the function of the lidar_process_thread_
   */
  void ProcessLidarPacketFn();
  /*
   * @brief push the received packets in the list
   * @param {struct} udp packet structure
   */
  void PushLidarData(const Packet& pkt);
  // the frame_id_ of the point cloud, useful in ROS
  std::string frame_id_;
  // the size of the lidar udp packet except header, used in "recvfrom"
  int packet_size_;
  // the lidar mark and serial like P40P VLP16
  std::string lidar_model_;
  // the list of the udp packets
  std::list<Packet> lidar_packet_list_;
  // udp input object
  boost::shared_ptr<InputBase> input_;
  // udp process object
  boost::shared_ptr<RawData> raw_data_;

  boost::shared_ptr<RawDataFactory> raw_data_factory_;
  // the mutex lock of the lidar list
  pthread_mutex_t lidar_lock_;
  // the semophore of the lidar packets receiver
  sem_t lidar_sem_;
  // is the threads run
  bool continue_lidar_recv_thread_;
  bool continue_lidar_process_thread_;
  // callback of the point cloud
  boost::function<void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, int timestamp)> user_lidar_callback_;
  // the two threads
  boost::thread *lidar_recv_thread_;
  boost::thread *lidar_process_thread_;
};

}  // namespace lidar_driver
}  // namespace itd_lidar

#endif  // _DRIVER_H_
