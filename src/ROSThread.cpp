#include <QMutexLocker>
#include <string>

#include "ROSThread.h"

using namespace std;

ROSThread::ROSThread(QObject *parent, QMutex *th_mutex) :
    QThread(parent), mutex_(th_mutex)
{
  processed_stamp_ = 0;
  play_rate_ = 1.0;
  loop_flag_ = false;
  save_flag_ = false;
  process_flag_ = false;
  stop_skip_flag_ = true;
  stereo_right_active_ = true;
  stereo_left_active_ = true;
  radarpolar_active_ = true;
  minimum = 0;
  search_bound_ = 10;
  // search_bound_ = 1000000;
  reset_process_stamp_flag_ = false;
  auto_start_flag_ = true;
  stamp_show_count_ = 0;
  imu_data_version_ = 0;
  prev_clock_stamp_ = 0;
}

ROSThread::~ROSThread()
{
  data_stamp_thread_.active_ = false;
  gps_thread_.active_ = false;
  inspva_thread_.active_ = false;
  imu_thread_.active_ = false;

  aeva_thread_.active_ = false;
  continental_thread_.active_ = false;
  continentalobject_thread_.active_ = false;
  radarpolar_thread_.active_ = false;
  stereo_right_thread_.active_ = false;
  stereo_left_thread_.active_ = false;

  usleep(100000);

  data_stamp_thread_.cv_.notify_all();
  if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();

  gps_thread_.cv_.notify_all();
  if(gps_thread_.thread_.joinable()) gps_thread_.thread_.join();
  
  inspva_thread_.cv_.notify_all();
  if(inspva_thread_.thread_.joinable()) inspva_thread_.thread_.join();

  imu_thread_.cv_.notify_all();
  if(imu_thread_.thread_.joinable()) imu_thread_.thread_.join();

  aeva_thread_.cv_.notify_all();
  if(aeva_thread_.thread_.joinable()) aeva_thread_.thread_.join();
  
  continental_thread_.cv_.notify_all();
  if(continental_thread_.thread_.joinable()) continental_thread_.thread_.join();

  continentalobject_thread_.cv_.notify_all();
  if(continentalobject_thread_.thread_.joinable()) continentalobject_thread_.thread_.join();

  radarpolar_thread_.cv_.notify_all(); 
  if(radarpolar_thread_.thread_.joinable()) radarpolar_thread_.thread_.join();

  stereo_right_thread_.cv_.notify_all();
  if(stereo_right_thread_.thread_.joinable()) stereo_right_thread_.thread_.join();

  stereo_left_thread_.cv_.notify_all();
  if(stereo_left_thread_.thread_.joinable()) stereo_left_thread_.thread_.join();

}

void ROSThread::ros_initialize(ros::NodeHandle &n)
{
  nh_ = n;

  pre_timer_stamp_ = ros::Time::now().toNSec();
  timer_ = nh_.createTimer(ros::Duration(0.0001), boost::bind(&ROSThread::TimerCallback, this, _1));

  start_sub_  = nh_.subscribe<std_msgs::Bool>("/hercules_file_player_start", 1, boost::bind(&ROSThread::FilePlayerStart, this, _1));
  stop_sub_    = nh_.subscribe<std_msgs::Bool>("/hercules_file_player_stop", 1, boost::bind(&ROSThread::FilePlayerStop, this, _1));

  gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1000);
  inspva_pub_ = nh_.advertise<novatel_gps_msgs::Inspva>("/inspva", 1000);

  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_raw", 1000);
  magnet_pub_ = nh_.advertise<sensor_msgs::MagneticField>("/imu/mag", 1000);

  aeva_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/aeva/points", 10000);

  continental_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/continental/points", 10000);
  continentalobject_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/continentalobject/points", 10000);

  radarpolar_pub_ = nh_.advertise<sensor_msgs::Image>("/radar/polar", 10); // giseop

  stereo_left_pub_ = nh_.advertise<sensor_msgs::Image>("/stereo/left/image_raw", 10);
  stereo_right_pub_ = nh_.advertise<sensor_msgs::Image>("/stereo/right/image_raw", 10);
  
  // stereo_left_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info", 10);
  // stereo_right_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info", 10);
 
  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
}

void ROSThread::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

void ROSThread::Ready()
{
  data_stamp_thread_.active_ = false;
  data_stamp_thread_.cv_.notify_all();
  if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();
  gps_thread_.active_ = false;
  gps_thread_.cv_.notify_all();
  if(gps_thread_.thread_.joinable()) gps_thread_.thread_.join();
  inspva_thread_.active_ = false;
  inspva_thread_.cv_.notify_all();
  if(inspva_thread_.thread_.joinable()) inspva_thread_.thread_.join();
  imu_thread_.active_ = false;
  imu_thread_.cv_.notify_all();
  if(imu_thread_.thread_.joinable()) imu_thread_.thread_.join();

  aeva_thread_.active_ = false;
  aeva_thread_.cv_.notify_all();
  if(aeva_thread_.thread_.joinable()) aeva_thread_.thread_.join();

  continental_thread_.active_ = false;
  continental_thread_.cv_.notify_all();
  if(continental_thread_.thread_.joinable()) continental_thread_.thread_.join();

  continentalobject_thread_.active_ = false;
  continentalobject_thread_.cv_.notify_all();
  if(continentalobject_thread_.thread_.joinable()) continentalobject_thread_.thread_.join();

  radarpolar_thread_.active_ = false;
  radarpolar_thread_.cv_.notify_all();
  if(radarpolar_thread_.thread_.joinable()) radarpolar_thread_.thread_.join();

  stereo_right_thread_.active_ = false;
  stereo_right_thread_.cv_.notify_all();
  if(stereo_right_thread_.thread_.joinable()) stereo_right_thread_.thread_.join();

  stereo_left_thread_.active_ = false;
  stereo_left_thread_.cv_.notify_all();
  if(stereo_left_thread_.thread_.joinable()) stereo_left_thread_.thread_.join();

  //check path is right or not
  ifstream f((data_folder_path_+"/sensor_data/datastamp.csv").c_str());
  if(!f.good()){
     cout << "Please check file path. Input path is wrong" << endl;
     return;
  }
  f.close();

  //Read CSV file and make map
  FILE *fp;
  int64_t stamp;
  //data stamp data load

  fp = fopen((data_folder_path_+"/sensor_data/datastamp.csv").c_str(),"r");
  char data_name[50];
  data_stamp_.clear();
  while(fscanf(fp,"%ld,%s\n",&stamp,data_name) == 2){
//    data_stamp_[stamp] = data_name;
    data_stamp_.insert( multimap<int64_t, string>::value_type(stamp, data_name));
  }
  cout << "Stamp data are loaded" << endl;
  fclose(fp);

  initial_data_stamp_ = data_stamp_.begin()->first - 1;
  last_data_stamp_ = prev(data_stamp_.end(),1)->first - 1;

//Read gps data
  fp = fopen((data_folder_path_+"/sensor_data/gps.csv").c_str(),"r");
  double latitude, longitude, altitude, altitude_orthometric;
  double cov[9];
  sensor_msgs::NavSatFix gps_data;
  gps_data_.clear();
  while( fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                &stamp,&latitude,&longitude,&altitude,&cov[0],&cov[1],&cov[2],&cov[3],&cov[4],&cov[5],&cov[6],&cov[7],&cov[8])
         == 13
         )
  {
    gps_data.header.stamp.fromNSec(stamp);
    gps_data.header.frame_id = "gps";
    gps_data.latitude = latitude;
    gps_data.longitude = longitude;
    gps_data.altitude = altitude;
    for(int i = 0 ; i < 9 ; i ++) gps_data.position_covariance[i] = cov[i];
    gps_data_[stamp] = gps_data;
  }
  cout << "Gps data are loaded" << endl;

  fclose(fp);




  //Read inspva data
  fp = fopen((data_folder_path_+"/sensor_data/inspva.csv").c_str(),"r");
  // double latitude, longitude, altitude, altitude_orthometric;
  double height, north_velocity, east_velocity, up_velocity, roll, pitch, azimuth;
  // string status;
  char status[17];
  int status_value;
  novatel_gps_msgs::Inspva inspva_data;
  inspva_data_.clear();
  while(fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%s %d",&stamp,&latitude,&longitude,&height,&north_velocity,&east_velocity,&up_velocity,&roll,&pitch,&azimuth,status, &status_value) == 12){
  //17%19[^\n] %29[^\n]
    
    inspva_data.header.stamp.fromNSec(stamp);
    inspva_data.header.frame_id = "inspva";
    inspva_data.latitude = latitude;
    inspva_data.longitude = longitude;
    inspva_data.height = height;
    inspva_data.north_velocity = north_velocity;
    inspva_data.east_velocity = east_velocity;
    inspva_data.up_velocity = up_velocity;
    inspva_data.roll = roll;
    inspva_data.pitch = pitch;
    inspva_data.azimuth = azimuth;
    inspva_data.status = std::string(status) + " " + std::to_string(status_value);
    inspva_data_[stamp] = inspva_data;
  }
  cout << "Inspva data are loaded" << endl;
  fclose(fp);

  //Read IMU data
  fp = fopen((data_folder_path_+"/sensor_data/xsens_imu.csv").c_str(),"r");
  double q_x,q_y,q_z,q_w,x,y,z,g_x,g_y,g_z,a_x,a_y,a_z,m_x,m_y,m_z;
  sensor_msgs::Imu imu_data;
  sensor_msgs::MagneticField mag_data;
  imu_data_.clear();
  mag_data_.clear();

  while(1){
    int length = fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",&stamp,&q_x,&q_y,&q_z,&q_w,&x,&y,&z,&g_x,&g_y,&g_z,&a_x,&a_y,&a_z,&m_x,&m_y,&m_z);
    if(length != 8 && length != 17) break;
    if(length == 8){
      imu_data.header.stamp.fromNSec(stamp);
      imu_data.header.frame_id = "imu";
      imu_data.orientation.x = q_x;
      imu_data.orientation.y = q_y;
      imu_data.orientation.z = q_z;
      imu_data.orientation.w = q_w;

      imu_data_[stamp] = imu_data;
      imu_data_version_ = 1;

    }else if(length == 17){
      imu_data.header.stamp.fromNSec(stamp);
      imu_data.header.frame_id = "imu";
      imu_data.orientation.x = q_x;
      imu_data.orientation.y = q_y;
      imu_data.orientation.z = q_z;
      imu_data.orientation.w = q_w;
      imu_data.angular_velocity.x = g_x;
      imu_data.angular_velocity.y = g_y;
      imu_data.angular_velocity.z = g_z;
      imu_data.linear_acceleration.x = a_x;
      imu_data.linear_acceleration.y = a_y;
      imu_data.linear_acceleration.z = a_z;

      imu_data.orientation_covariance[0] = 3;
      imu_data.orientation_covariance[4] = 3;
      imu_data.orientation_covariance[8] = 3;
      imu_data.angular_velocity_covariance[0] = 3;
      imu_data.angular_velocity_covariance[4] = 3;
      imu_data.angular_velocity_covariance[8] = 3;
      imu_data.linear_acceleration_covariance[0] = 3;
      imu_data.linear_acceleration_covariance[4] = 3;
      imu_data.linear_acceleration_covariance[8] = 3;


      imu_data_[stamp] = imu_data;
      mag_data.header.stamp.fromNSec(stamp);
      mag_data.header.frame_id = "imu";
      mag_data.magnetic_field.x = m_x;
      mag_data.magnetic_field.y = m_y;
      mag_data.magnetic_field.z = m_z;
      mag_data_[stamp] = mag_data;
      imu_data_version_ = 2;
    }
  }
  cout << "IMU data are loaded" << endl;
  fclose(fp);

  aeva_file_list_.clear();
  continental_file_list_.clear();
  continentalobject_file_list_.clear();
  radarpolar_file_list_.clear();
  stereo_right_file_list_.clear();
  stereo_left_file_list_.clear();

  // GetDirList(data_folder_path_ + "/image/stereo_left",stereo_file_list_);
  GetDirList(data_folder_path_ + "/LiDAR/Aeva",aeva_file_list_);
  GetDirList(data_folder_path_ + "/Radar/Continental",continental_file_list_);
  GetDirList(data_folder_path_ + "/Radar/Continentalobject",continentalobject_file_list_);
  GetDirList(data_folder_path_ + "/Radar/Navtech", radarpolar_file_list_);
  GetDirList(data_folder_path_ + "/Image/stereo_right", stereo_right_file_list_);
  GetDirList(data_folder_path_ + "/Image/stereo_left", stereo_left_file_list_);

  //load camera info

  // left_camera_nh_ = ros::NodeHandle(nh_,"left");
  // right_camera_nh_ = ros::NodeHandle(nh_,"right");

  // left_cinfo_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(left_camera_nh_,"/stereo/left"));
  // right_cinfo_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(right_camera_nh_,"/stereo/right"));

  // string left_yaml_file_path = "file://" + data_folder_path_ + "/calibration/left.yaml";
  // string right_yaml_file_path = "file://" + data_folder_path_ + "/calibration/right.yaml";


  // if(left_cinfo_->validateURL(left_yaml_file_path)){
  //     left_cinfo_->loadCameraInfo(left_yaml_file_path);
  //    cout << "Success to load camera info" << endl;
  //     stereo_left_info_ = left_cinfo_->getCameraInfo();
  // }


  // if(right_cinfo_->validateURL(right_yaml_file_path)){
  //     right_cinfo_->loadCameraInfo(right_yaml_file_path);
  //    cout << "Success to load camera info" << endl;
  //     stereo_right_info_ = right_cinfo_->getCameraInfo();
  // }


  data_stamp_thread_.active_ = true;
  gps_thread_.active_ = true;
  inspva_thread_.active_ = true;
  imu_thread_.active_ = true;
  aeva_thread_.active_ = true;
  continental_thread_.active_ = true;
  continentalobject_thread_.active_ = true;
  radarpolar_thread_.active_ = true;
  stereo_right_thread_.active_ = true;
  stereo_left_thread_.active_ = true;


  data_stamp_thread_.thread_ = std::thread(&ROSThread::DataStampThread,this);
  gps_thread_.thread_ = std::thread(&ROSThread::GpsThread,this);
  inspva_thread_.thread_ = std::thread(&ROSThread::InspvaThread,this);
  imu_thread_.thread_ = std::thread(&ROSThread::ImuThread,this);
  aeva_thread_.thread_ = std::thread(&ROSThread::AevaThread,this);
  continental_thread_.thread_ = std::thread(&ROSThread::ContinentalThread,this);
  continentalobject_thread_.thread_ = std::thread(&ROSThread::ContinentalobjectThread,this);
  radarpolar_thread_.thread_ = std::thread(&ROSThread::RadarpolarThread,this);
  stereo_right_thread_.thread_ = std::thread(&ROSThread::StereorightThread,this);
  stereo_left_thread_.thread_ = std::thread(&ROSThread::StereoleftThread,this);
}

void ROSThread::DataStampThread()
{
  auto stop_region_iter = stop_period_.begin();

  for(auto iter = data_stamp_.begin() ; iter != data_stamp_.end() ; iter ++){
    auto stamp = iter->first;

    while((stamp > (initial_data_stamp_+processed_stamp_))&&(data_stamp_thread_.active_ == true)){
      if(processed_stamp_ == 0){
          iter = data_stamp_.begin();
          stop_region_iter = stop_period_.begin();
          stamp = iter->first;
      }
      usleep(1);
      if(reset_process_stamp_flag_ == true) break;
      //wait for data publish
    }

    if(reset_process_stamp_flag_ == true){
      auto target_stamp = processed_stamp_ + initial_data_stamp_;
      //set iter
      iter = data_stamp_.lower_bound(target_stamp);
      iter = prev(iter,1);
      //set stop region order
      auto new_stamp = iter->first;
      stop_region_iter = stop_period_.upper_bound(new_stamp);

      reset_process_stamp_flag_ = false;
      continue;
    }


    //check whether stop region or not
    if(stamp == stop_region_iter->first){
      if(stop_skip_flag_ == true){
        cout << "Skip stop section!!" << endl;
        iter = data_stamp_.find(stop_region_iter->second);  //find stop region end
        iter = prev(iter,1);
        processed_stamp_ = stop_region_iter->second - initial_data_stamp_;
      }
      stop_region_iter++;
      if(stop_skip_flag_ == true){
        continue;
      }
    }

    if(data_stamp_thread_.active_ == false) return;
    if(iter->second.compare("gps") == 0){
      gps_thread_.push(stamp);
      gps_thread_.cv_.notify_all();
    }else if(iter->second.compare("gps") == 0){
      gps_thread_.push(stamp);
      gps_thread_.cv_.notify_all();
    }
    else if(iter->second.compare("inspva") == 0){
      inspva_thread_.push(stamp);
      inspva_thread_.cv_.notify_all();
    }else if(iter->second.compare("imu") == 0){
      imu_thread_.push(stamp);
      imu_thread_.cv_.notify_all();
    }else if(iter->second.compare("aeva") == 0){
        aeva_thread_.push(stamp);
        aeva_thread_.cv_.notify_all();
    }else if(iter->second.compare("continental") == 0){
        continental_thread_.push(stamp);
        continental_thread_.cv_.notify_all();
    }else if(iter->second.compare("continentalobject") == 0){
        continentalobject_thread_.push(stamp);
        continentalobject_thread_.cv_.notify_all();
    }else if(iter->second.compare("navtech") == 0 && radarpolar_active_ == true){
        radarpolar_thread_.push(stamp);
        radarpolar_thread_.cv_.notify_all();
    }else if(iter->second.compare("stereo_right") == 0 && stereo_right_active_ == true){
        stereo_right_thread_.push(stamp);
        stereo_right_thread_.cv_.notify_all();
    }else if(iter->second.compare("stereo_left") == 0 && stereo_left_active_ == true){
        stereo_left_thread_.push(stamp);
        stereo_left_thread_.cv_.notify_all();
    }
    stamp_show_count_++;
    if(stamp_show_count_ > 100){
      stamp_show_count_ = 0;
      emit StampShow(stamp);
    }

    if(prev_clock_stamp_ == 0 || (stamp - prev_clock_stamp_) > 10000000){
        rosgraph_msgs::Clock clock;
        clock.clock.fromNSec(stamp);
        clock_pub_.publish(clock);
        prev_clock_stamp_ = stamp;
    }

    if(loop_flag_ == true && iter == prev(data_stamp_.end(),1)){
        iter = data_stamp_.begin();
        stop_region_iter = stop_period_.begin();
        processed_stamp_ = 0;
    }
    if(loop_flag_ == false && iter == prev(data_stamp_.end(),1)){
        play_flag_ = false;
        while(!play_flag_){
            iter = data_stamp_.begin();
            stop_region_iter = stop_period_.begin();
            processed_stamp_ = 0;
            usleep(10000);
        }
    }
    if(save_flag_ == true && process_flag_ == false){
      bag_.open(data_folder_path_ + "/" + to_string(bag_idx_) + ".bag", rosbag::bagmode::Write);
      process_flag_ = true;
    }
    else if(save_flag_ == false && process_flag_ == true){
      process_flag_ = false;
      bag_.close();
      bag_idx_++;
    }
  }
  cout << "Data publish complete" << endl;

}

void ROSThread::GpsThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(gps_thread_.mutex_);
    gps_thread_.cv_.wait(ul);
    if(gps_thread_.active_ == false) return;
    ul.unlock();

    while(!gps_thread_.data_queue_.empty()){
      auto data = gps_thread_.pop();
      //process
      if(gps_data_.find(data) != gps_data_.end()){
        gps_pub_.publish(gps_data_[data]);
      }
    }
    if(gps_thread_.active_ == false) return;
  }
}

void ROSThread::InspvaThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(inspva_thread_.mutex_);
    inspva_thread_.cv_.wait(ul);
    if(inspva_thread_.active_ == false) return;
    ul.unlock();
    while(!inspva_thread_.data_queue_.empty()){
      auto data = inspva_thread_.pop();
      //process
      if(inspva_data_.find(data) != inspva_data_.end()){
        if(save_flag_ == true && process_flag_ == true){
          std::lock_guard<std::mutex> lock(bag_mutex_);
          bag_.write("/inspva", inspva_data_[data].header.stamp, inspva_data_[data]);
        }
        inspva_pub_.publish(inspva_data_[data]);
      }

    }
    if(inspva_thread_.active_ == false) return;
  }
}

void ROSThread::ImuThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(imu_thread_.mutex_);
    imu_thread_.cv_.wait(ul);
    if(imu_thread_.active_ == false) return;
    ul.unlock();
    while(!imu_thread_.data_queue_.empty()){
      auto data = imu_thread_.pop();
      //process
      if(imu_data_.find(data) != imu_data_.end()){
        if(save_flag_ == true && process_flag_ == true){
          std::lock_guard<std::mutex> lock(bag_mutex_);
          bag_.write("/imu/data_raw", imu_data_[data].header.stamp, imu_data_[data]);
        }
        imu_pub_.publish(imu_data_[data]);
      }

    }
    if(imu_thread_.active_ == false) return;
  }
}

void ROSThread::TimerCallback(const ros::TimerEvent&)
{
    int64_t current_stamp = ros::Time::now().toNSec();
    if(play_flag_ == true && pause_flag_ == false){
      processed_stamp_ += static_cast<int64_t>(static_cast<double>(current_stamp - pre_timer_stamp_) * play_rate_);
    }
    pre_timer_stamp_ = current_stamp;

    if(play_flag_ == false){
      processed_stamp_ = 0; //reset
      prev_clock_stamp_ = 0;
    }
}


void ROSThread::AevaThread()
{
  int current_file_index = 0;
  int previous_file_index = 0;
  while(1){
    std::unique_lock<std::mutex> ul(aeva_thread_.mutex_);
    aeva_thread_.cv_.wait(ul);
    if(aeva_thread_.active_ == false) return;
    ul.unlock();
std::cout.precision(20);
    while(!aeva_thread_.data_queue_.empty()){
      auto data = aeva_thread_.pop();
      //process
      //publish data
      if(to_string(data) + ".bin" == aeva_next_.first){
        //publish
        aeva_next_.second.header.stamp.fromNSec(data) ;
        aeva_next_.second.header.frame_id = "aeva";
        if(save_flag_ == true && process_flag_ == true){
          std::lock_guard<std::mutex> lock(bag_mutex_);
          bag_.write("/aeva/points", aeva_next_.second.header.stamp, aeva_next_.second);
        }
        aeva_pub_.publish(aeva_next_.second);

      }else{
        //load current data
        pcl::PointCloud<pc_type_a> cloud;
        cloud.clear();
        sensor_msgs::PointCloud2 publish_cloud;
        string current_file_name = data_folder_path_ + "/LiDAR/Aeva" +"/"+ to_string(data) + ".bin";
        if(find(next(aeva_file_list_.begin(),max(0,previous_file_index-search_bound_)),aeva_file_list_.end(),to_string(data)+".bin") != aeva_file_list_.end()){
            ifstream file;
            file.open(current_file_name, ios::in|ios::binary);
            while(!file.eof()){
                pc_type_a point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.velocity), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.time_offset_ns), sizeof(int32_t));
                file.read(reinterpret_cast<char *>(&point.line_index), sizeof(uint8_t));
                if(data > 1691936557946849179)
                    file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                cloud.points.push_back (point);
            }
            file.close();

            pcl::toROSMsg(cloud, publish_cloud);
            publish_cloud.header.stamp.fromNSec(data); 
            publish_cloud.header.frame_id = "aeva";          
            aeva_pub_.publish(publish_cloud);

        }
        previous_file_index = 0;
      }

      //load next data
      pcl::PointCloud<pc_type_a> cloud;
      cloud.clear();
      sensor_msgs::PointCloud2 publish_cloud;
      current_file_index = find(next(aeva_file_list_.begin(),max(0,previous_file_index-search_bound_)),aeva_file_list_.end(),to_string(data)+".bin") - aeva_file_list_.begin();
      if(find(next(aeva_file_list_.begin(),max(0,previous_file_index-search_bound_)),aeva_file_list_.end(),aeva_file_list_[current_file_index+1]) != aeva_file_list_.end()){
          string next_file_name = data_folder_path_ + "/LiDAR/Aeva" +"/"+ aeva_file_list_[current_file_index+1];

          ifstream file;
          file.open(next_file_name, ios::in|ios::binary);
          while(!file.eof()){
              pc_type_a point;
              file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.velocity), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.time_offset_ns), sizeof(int32_t));
              file.read(reinterpret_cast<char *>(&point.line_index), sizeof(uint8_t));
              if(data > 1691936557946849179)
                  file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
              cloud.points.push_back (point);
          }
          file.close();
          pcl::toROSMsg(cloud, publish_cloud);
          aeva_next_ = make_pair(aeva_file_list_[current_file_index+1], publish_cloud);
      }

      previous_file_index = current_file_index;
    }
    if(aeva_thread_.active_ == false) return;
  }
}

void ROSThread::ContinentalThread()
{
    int current_file_index = 0;
    int previous_file_index = 0;
    while(1){
        std::unique_lock<std::mutex> ul(continental_thread_.mutex_);
        continental_thread_.cv_.wait(ul);
        if(continental_thread_.active_ == false) return;
        ul.unlock();
        std::cout.precision(20);
        
        while(!continental_thread_.data_queue_.empty()){
            auto data = continental_thread_.pop();
            //process
            //publish data
            if(to_string(data) + ".bin" == continental_next_.first){
                //publish
                continental_next_.second.header.stamp.fromNSec(data);
                continental_next_.second.header.frame_id = "continental";
                if(save_flag_ == true && process_flag_ == true){
                    std::lock_guard<std::mutex> lock(bag_mutex_);
                    bag_.write("/continental/points", continental_next_.second.header.stamp, continental_next_.second);
                }
                continental_pub_.publish(continental_next_.second);

            } else {
                //load current data
                pcl::PointCloud<pc_type_c> cloud;
                cloud.clear();
                sensor_msgs::PointCloud2 publish_cloud;
                string current_file_name = data_folder_path_ + "/Radar/Continental" + "/" + to_string(data) + ".bin";
                if(find(next(continental_file_list_.begin(), max(0, previous_file_index - search_bound_)), continental_file_list_.end(), to_string(data) + ".bin") != continental_file_list_.end()){
                    ifstream file;
                    file.open(current_file_name, ios::in|ios::binary);
                    while(!file.eof()){
                        pc_type_c point;
                        file.read(reinterpret_cast<char *>(&point.x), sizeof(point.x));
                        file.read(reinterpret_cast<char *>(&point.y), sizeof(point.y));
                        file.read(reinterpret_cast<char *>(&point.z), sizeof(point.z));
                        file.read(reinterpret_cast<char *>(&point.v), sizeof(point.v));
                        file.read(reinterpret_cast<char *>(&point.r), sizeof(point.r));
                        file.read(reinterpret_cast<char *>(&point.RCS), sizeof(point.RCS));
                        file.read(reinterpret_cast<char *>(&point.azimuth), sizeof(point.azimuth));
                        file.read(reinterpret_cast<char *>(&point.elevation), sizeof(point.elevation));
                        cloud.points.push_back(point);
                    }
                    file.close();

                    pcl::toROSMsg(cloud, publish_cloud);
                    publish_cloud.header.stamp.fromNSec(data); 
                    publish_cloud.header.frame_id = "continental";          
                    continental_pub_.publish(publish_cloud);
                }
                previous_file_index = 0;
            }

            //load next data
            pcl::PointCloud<pc_type_c> cloud;
            cloud.clear();
            sensor_msgs::PointCloud2 publish_cloud;
            current_file_index = find(next(continental_file_list_.begin(), max(0, previous_file_index - search_bound_)), continental_file_list_.end(), to_string(data) + ".bin") - continental_file_list_.begin();
            if(find(next(continental_file_list_.begin(), max(0, previous_file_index - search_bound_)), continental_file_list_.end(), continental_file_list_[current_file_index + 1]) != continental_file_list_.end()){
                string next_file_name = data_folder_path_ + "/Radar/Continental" + "/" + continental_file_list_[current_file_index + 1];

                ifstream file;
                file.open(next_file_name, ios::in|ios::binary);
                while(!file.eof()){
                    pc_type_c point;
                    file.read(reinterpret_cast<char *>(&point.x), sizeof(point.x));
                    file.read(reinterpret_cast<char *>(&point.y), sizeof(point.y));
                    file.read(reinterpret_cast<char *>(&point.z), sizeof(point.z));
                    file.read(reinterpret_cast<char *>(&point.v), sizeof(point.v));
                    file.read(reinterpret_cast<char *>(&point.r), sizeof(point.r));
                    file.read(reinterpret_cast<char *>(&point.RCS), sizeof(point.RCS));
                    file.read(reinterpret_cast<char *>(&point.azimuth), sizeof(point.azimuth));
                    file.read(reinterpret_cast<char *>(&point.elevation), sizeof(point.elevation));
                    cloud.points.push_back(point);
                }
                file.close();
                pcl::toROSMsg(cloud, publish_cloud);
                continental_next_ = make_pair(continental_file_list_[current_file_index + 1], publish_cloud);
            }

            previous_file_index = current_file_index;
        }
        if(continental_thread_.active_ == false) return;
    }
}

void ROSThread::ContinentalobjectThread()
{
    int current_file_index = 0;
    int previous_file_index = 0;
    while(1){
        std::unique_lock<std::mutex> ul(continentalobject_thread_.mutex_);
        continentalobject_thread_.cv_.wait(ul);
        if(continentalobject_thread_.active_ == false) return;
        ul.unlock();
        std::cout.precision(20);
        
        while(!continentalobject_thread_.data_queue_.empty()){
            auto data = continentalobject_thread_.pop();
            //process
            //publish data
            if(to_string(data) + ".bin" == continentalobject_next_.first){
                //publish
                continentalobject_next_.second.header.stamp.fromNSec(data);
                continentalobject_next_.second.header.frame_id = "continentalobject";
                if(save_flag_ == true && process_flag_ == true){
                    std::lock_guard<std::mutex> lock(bag_mutex_);
                    bag_.write("/continentalobject/points", continentalobject_next_.second.header.stamp, continentalobject_next_.second);
                }
                continentalobject_pub_.publish(continentalobject_next_.second);

            } else {
                //load current data
                pcl::PointCloud<pc_type_co> cloud;
                cloud.clear();
                sensor_msgs::PointCloud2 publish_cloud;
                string current_file_name = data_folder_path_ + "/Radar/Continentalobject" + "/" + to_string(data) + ".bin";
                if(find(next(continentalobject_file_list_.begin(), max(0, previous_file_index - search_bound_)), continentalobject_file_list_.end(), to_string(data) + ".bin") != continentalobject_file_list_.end()){
                    ifstream file;
                    file.open(current_file_name, ios::in|ios::binary);
                    while(!file.eof()){
                        pc_type_co point;
                        file.read(reinterpret_cast<char *>(&point.x), sizeof(point.x));
                        file.read(reinterpret_cast<char *>(&point.y), sizeof(point.y));
                        file.read(reinterpret_cast<char *>(&point.z), sizeof(point.z));
                        file.read(reinterpret_cast<char *>(&point.vx), sizeof(point.vx));
                        file.read(reinterpret_cast<char *>(&point.vy), sizeof(point.vy));
                        cloud.points.push_back(point);
                    }
                    file.close();

                    pcl::toROSMsg(cloud, publish_cloud);
                    publish_cloud.header.stamp.fromNSec(data); 
                    publish_cloud.header.frame_id = "continentalobject";          
                    continentalobject_pub_.publish(publish_cloud);
                }
                previous_file_index = 0;
            }

            //load next data
            pcl::PointCloud<pc_type_co> cloud;
            cloud.clear();
            sensor_msgs::PointCloud2 publish_cloud;
            current_file_index = find(next(continentalobject_file_list_.begin(), max(0, previous_file_index - search_bound_)), continentalobject_file_list_.end(), to_string(data) + ".bin") - continentalobject_file_list_.begin();
            if(find(next(continentalobject_file_list_.begin(), max(0, previous_file_index - search_bound_)), continentalobject_file_list_.end(), continentalobject_file_list_[current_file_index + 1]) != continentalobject_file_list_.end()){
                string next_file_name = data_folder_path_ + "/Radar/Continentalobject" + "/" + continentalobject_file_list_[current_file_index + 1];

                ifstream file;
                file.open(next_file_name, ios::in|ios::binary);
                while(!file.eof()){
                    pc_type_co point;
                    file.read(reinterpret_cast<char *>(&point.x), sizeof(point.x));
                    file.read(reinterpret_cast<char *>(&point.y), sizeof(point.y));
                    file.read(reinterpret_cast<char *>(&point.z), sizeof(point.z));
                    file.read(reinterpret_cast<char *>(&point.vx), sizeof(point.vx));
                    file.read(reinterpret_cast<char *>(&point.vy), sizeof(point.vy));
                    cloud.points.push_back(point);
                }
                file.close();
                pcl::toROSMsg(cloud, publish_cloud);
                continentalobject_next_ = make_pair(continentalobject_file_list_[current_file_index + 1], publish_cloud);
            }

            previous_file_index = current_file_index;
        }
        if(continentalobject_thread_.active_ == false) return;
    }
}


void 
ROSThread::RadarpolarThread()
{
  int current_img_index = 0;
  int previous_img_index = 0;

  while(1){
    std::unique_lock<std::mutex> ul(radarpolar_thread_.mutex_);
    radarpolar_thread_.cv_.wait(ul);
    if(radarpolar_thread_.active_ == false)
      return;
    ul.unlock();

    while(!radarpolar_thread_.data_queue_.empty())
    {
      auto data = radarpolar_thread_.pop();
      //process
      if(radarpolar_file_list_.size() == 0) continue;

      //publish
      if( to_string(data)+".png" == radarpolar_next_.first && !radarpolar_next_.second.empty() )
      {
        cv_bridge::CvImage radarpolar_out_msg;
        radarpolar_out_msg.header.stamp.fromNSec(data);
        radarpolar_out_msg.header.frame_id = "navtech";
        radarpolar_out_msg.encoding = sensor_msgs::image_encodings::MONO8;
        radarpolar_out_msg.image    = radarpolar_next_.second;
        radarpolar_pub_.publish(radarpolar_out_msg.toImageMsg());
      }
      else
      {
        string current_radarpolar_name = data_folder_path_ + "/Radar/Navtech" + "/" + to_string(data) + ".png";

        cv::Mat radarpolar_image;
        radarpolar_image = imread(current_radarpolar_name, cv::IMREAD_GRAYSCALE);
        if(!radarpolar_image.empty())
        {

          cv_bridge::CvImage radarpolar_out_msg;
          radarpolar_out_msg.header.stamp.fromNSec(data);
          radarpolar_out_msg.header.frame_id = "navtech";
          radarpolar_out_msg.encoding = sensor_msgs::image_encodings::MONO8;
          radarpolar_out_msg.image    = radarpolar_image;
          radarpolar_pub_.publish(radarpolar_out_msg.toImageMsg());

        }
        previous_img_index = 0;
      }

      //load next image
      current_img_index = find( next(radarpolar_file_list_.begin(),max(0,previous_img_index - search_bound_)), radarpolar_file_list_.end(), to_string(data)+".png" ) - radarpolar_file_list_.begin();
      if(current_img_index < radarpolar_file_list_.size()-2)
      {
        string next_radarpolar_name = data_folder_path_ + "/Radar/Navtech" +"/"+ radarpolar_file_list_[current_img_index+1];

        cv::Mat radarpolar_image;
        radarpolar_image = imread(next_radarpolar_name, cv::IMREAD_COLOR);

        if(!radarpolar_image.empty())
        {
          cv::cvtColor(radarpolar_image, radarpolar_image, cv::COLOR_RGB2BGR);
          radarpolar_next_ = make_pair(radarpolar_file_list_[current_img_index+1], radarpolar_image);
        }
      }
      previous_img_index = current_img_index;
    }
    
    if(radarpolar_thread_.active_ == false) return;
  }
}

// StereoleftThread
void ROSThread::StereoleftThread()
{
    int current_img_index = 0;
    int previous_img_index = 0;

    while (1) {
        std::unique_lock<std::mutex> ul(stereo_left_thread_.mutex_);
        stereo_left_thread_.cv_.wait(ul);
        if (stereo_left_thread_.active_ == false) return;
        ul.unlock();

        while (!stereo_left_thread_.data_queue_.empty()) {
            auto data = stereo_left_thread_.pop();
            // Process
            if (stereo_left_file_list_.empty()) continue;

            // Publish
            if (to_string(data) + ".png" == stereo_left_next_img_.first && !stereo_left_next_img_.second.empty()) {
                cv_bridge::CvImage left_out_msg;
                left_out_msg.header.stamp.fromNSec(data);
                left_out_msg.header.frame_id = "stereo_left";
                left_out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Change to BGR8
                left_out_msg.image = stereo_left_next_img_.second;

                stereo_left_pub_.publish(left_out_msg.toImageMsg());
            } else {
                // Load left stereo image
                string current_stereo_left_name = data_folder_path_ + "/Image/stereo_left/" + to_string(data) + ".png";
                cv::Mat current_left_image = imread(current_stereo_left_name, cv::IMREAD_COLOR); // Load as color image

                if (!current_left_image.empty()) {
                    cv_bridge::CvImage left_out_msg;
                    left_out_msg.header.stamp.fromNSec(data);
                    left_out_msg.header.frame_id = "stereo_left";
                    left_out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Change to BGR8
                    left_out_msg.image = current_left_image;

                    stereo_left_pub_.publish(left_out_msg.toImageMsg());
                }
                previous_img_index = 0;
            }

            // Load next left image
            current_img_index = find(next(stereo_left_file_list_.begin(), max(0, previous_img_index - search_bound_)), stereo_left_file_list_.end(), to_string(data) + ".png") - stereo_left_file_list_.begin();
            if (current_img_index < stereo_left_file_list_.size() - 1) {
                string next_stereo_left_name = data_folder_path_ + "/Image/stereo_left/" + stereo_left_file_list_[current_img_index + 1];
                cv::Mat next_left_image = imread(next_stereo_left_name, cv::IMREAD_COLOR); // Load as color image
                if (!next_left_image.empty()) {
                    stereo_left_next_img_ = make_pair(stereo_left_file_list_[current_img_index + 1], next_left_image);
                }
            }
            previous_img_index = current_img_index;
        }
        if (stereo_left_thread_.active_ == false) return;
    }
}

// StereorightThread
void ROSThread::StereorightThread()
{
    int current_img_index = 0;
    int previous_img_index = 0;

    while (1) {
        std::unique_lock<std::mutex> ul(stereo_right_thread_.mutex_);
        stereo_right_thread_.cv_.wait(ul);
        if (stereo_right_thread_.active_ == false) return;
        ul.unlock();

        while (!stereo_right_thread_.data_queue_.empty()) {
            auto data = stereo_right_thread_.pop();
            // Process
            if (stereo_right_file_list_.empty()) continue;

            // Publish
            if (to_string(data) + ".png" == stereo_right_next_img_.first && !stereo_right_next_img_.second.empty()) {
                cv_bridge::CvImage right_out_msg;
                right_out_msg.header.stamp.fromNSec(data);
                right_out_msg.header.frame_id = "stereo_right";
                right_out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Change to BGR8
                right_out_msg.image = stereo_right_next_img_.second;

                stereo_right_pub_.publish(right_out_msg.toImageMsg());
            } else {
                // Load right stereo image
                string current_stereo_right_name = data_folder_path_ + "/Image/stereo_right/" + to_string(data) + ".png";
                cv::Mat current_right_image = imread(current_stereo_right_name, cv::IMREAD_COLOR); // Load as color image

                if (!current_right_image.empty()) {
                    cv_bridge::CvImage right_out_msg;
                    right_out_msg.header.stamp.fromNSec(data);
                    right_out_msg.header.frame_id = "stereo_right";
                    right_out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Change to BGR8
                    right_out_msg.image = current_right_image;

                    stereo_right_pub_.publish(right_out_msg.toImageMsg());
                }
                previous_img_index = 0;
            }

            // Load next right image
            current_img_index = find(next(stereo_right_file_list_.begin(), max(0, previous_img_index - search_bound_)), stereo_right_file_list_.end(), to_string(data) + ".png") - stereo_right_file_list_.begin();
            if (current_img_index < stereo_right_file_list_.size() - 1) {
                string next_stereo_right_name = data_folder_path_ + "/Image/stereo_right/" + stereo_right_file_list_[current_img_index + 1];
                cv::Mat next_right_image = imread(next_stereo_right_name, cv::IMREAD_COLOR); // Load as color image
                if (!next_right_image.empty()) {
                    stereo_right_next_img_ = make_pair(stereo_right_file_list_[current_img_index + 1], next_right_image);
                }
            }
            previous_img_index = current_img_index;
        }
        if (stereo_right_thread_.active_ == false) return;
    }
}


int ROSThread::GetDirList(string dir, vector<string> &files)
{

  vector<string> tmp_files;
  struct dirent **namelist;
  int n;
  n = scandir(dir.c_str(),&namelist, 0 , alphasort);
  if (n < 0)
      perror("scandir");
  else {
      while (n--) {
      if(string(namelist[n]->d_name) != "." && string(namelist[n]->d_name) != ".."){
        tmp_files.push_back(string(namelist[n]->d_name));
      }
      free(namelist[n]);
      }
      free(namelist);
  }

  for(auto iter = tmp_files.rbegin() ; iter!= tmp_files.rend() ; iter++){
    files.push_back(*iter);
  }
    return 0;
}

void ROSThread::FilePlayerStart(const std_msgs::BoolConstPtr& msg)
{
  if(auto_start_flag_ == true){
    cout << "File player auto start" << endl;
    usleep(1000000);
    play_flag_ = false;
    emit StartSignal();
  }
}

void ROSThread::FilePlayerStop(const std_msgs::BoolConstPtr& msg)
{
  cout << "File player auto stop" << endl;
  play_flag_ = true;
  emit StartSignal();
}
void ROSThread::ResetProcessStamp(int position)
{
  if(position > 0 && position < 10000){
    processed_stamp_ = static_cast<int64_t>(static_cast<float>(last_data_stamp_ - initial_data_stamp_)*static_cast<float>(position)/static_cast<float>(10000));
    reset_process_stamp_flag_ = true;
  }
}
