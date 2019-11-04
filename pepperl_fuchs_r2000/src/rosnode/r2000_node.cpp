// Copyright (c) 2014-2017, Pepperl+Fuchs GmbH, Mannheim
// Copyright (c) 2014-2017, Denis Dillenberger
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice, this
//   list of conditions and the following disclaimer in the documentation and/or
//   other materials provided with the distribution.
//
// * Neither the name of Pepperl+Fuchs nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "r2000_node.h"
#include <sensor_msgs/LaserScan.h>
#include <pepperl_fuchs_r2000/r2000_driver.h>

namespace pepperl_fuchs {

//-----------------------------------------------------------------------------
R2000Node::R2000Node():nh_("~")
{
    driver_ = 0;
    // Reading and checking parameters
    //-------------------------------------------------------------------------
    nh_.param("frame_id", frame_id_, std::string("/scan"));
    nh_.param("scanner_ip",scanner_ip_,std::string(""));
    nh_.param("scan_frequency",scan_frequency_,35);
    nh_.param("samples_per_scan",samples_per_scan_,3600);
    nh_.param("start_angle",start_angle_,-1800000);
    nh_.param("max_num_points_scan",max_num_points_scan_,0);
    nh_.param("hmi_application_bitmap",hmi_application_bitmap_,std::string(""));
    nh_.param("hmi_display_mode",hmi_display_mode_,std::string(""));
    nh_.param("latency_offset",latency_offset_,0.0);
    nh_.param("synchronize_time",synchronize_time_,true);

    if( scanner_ip_ == "" )
    {
        std::cerr << "IP of laser range finder not set!" << std::endl;
        return;
    }

    if( !connect() )
        return;

    // Declare publisher and create timer
    //-------------------------------------------------------------------------
    scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("scan",100);
    cmd_subscriber_ = nh_.subscribe("control_command",100,&R2000Node::cmdMsgCallback,this);
    //get_scan_data_timer_ = nh_.createTimer(ros::Duration(1/(2*std::atof(driver_->getParametersCached().at("scan_frequency").c_str()))), &R2000Node::getScanData, this);

    std::cout << "Starting measurement pooling" << std::endl;
    while (ros::ok())
    {
      getScanData();
      ros::spinOnce();
    }
}

R2000Node::~R2000Node()
{
  ROS_INFO("Disconnecting R2000");
  driver_->disconnect();
}

//-----------------------------------------------------------------------------
bool R2000Node::connect()
{
    delete driver_;

    // Connecting to laser range finder
    //-------------------------------------------------------------------------
    driver_ = new R2000Driver();
    std::cout << "Connecting to scanner at " << scanner_ip_ << " ... ";
    if( driver_->connect(scanner_ip_,80) )
        std::cout << "OK" << std::endl;
    else
    {
        std::cout << "FAILED!" << std::endl;
        std::cerr << "Connection to scanner at " << scanner_ip_ << " failed!" << std::endl;
        return false;
    }

    // Setting, reading and displaying parameters
    //-------------------------------------------------------------------------
    driver_->setScanFrequency(scan_frequency_);
    driver_->setSamplesPerScan(samples_per_scan_);

    auto params = driver_->getParameters();
    if( hmi_application_bitmap_ != "" )
    {
        driver_->setParameter("hmi_application_bitmap", hmi_application_bitmap_);
        if( driver_->getParametersCached().at("hmi_display_mode") != "application_bitmap" )
        {
            driver_->setParameter("hmi_display_mode","application_bitmap");
        }
        params = driver_->getParameters();
    }
    if( hmi_display_mode_ != "" )
    {
        if( driver_->getParametersCached().at("hmi_display_mode") != hmi_display_mode_ )
        {
            driver_->setParameter("hmi_display_mode",hmi_display_mode_);
            params = driver_->getParameters();
        }
    }

    std::cout << "Current scanner settings:" << std::endl;
    std::cout << "============================================================" << std::endl;
    for( const auto& p : params )
        std::cout << p.first << " : " << p.second << std::endl;
    std::cout << "============================================================" << std::endl;


    std::cout << "setting I/Q parameters" << std::endl;
    if (!driver_->setIQPolarity(2, "active_low"))
    {
        std::cerr << "Could not set I/Q polarity";
    }
    if (!driver_->setIQOffDelay(2, 1))
    {
        std::cerr << "Could not set I/Q off_delay";
    }
    if (!driver_->setIQMode(2, "output_push_pull"))
    {
        std::cerr << "Could not set I/Q mode";
    }
    if (!driver_->setIQSource(2, "timesync"))
    {
        std::cerr << "Could not set I/Q source";
    }
    if (!driver_->setIQTimesyncInterval(1000))
    {
        std::cerr << "Could not set I/Q timesync interval";
    }
    if (!driver_->setIQGlobalEnable(true))
    {
        std::cerr << "Could not set I/Q global enable";
    }
//    // Clock sync procedure
//    //-------------------------------------------------------------------------
//    ros::Time before_sync_time=ros::Time::now();
//    int64_t offset_nano=0;
//    int number_of_sync_measurements=100;
//    double measurement_duration;

//    for (int i = 0; i < number_of_sync_measurements; i++ ){
//      ros::Time time_before_measurement=ros::Time::now();
//      std::string raw_lidar_time = driver_->getParameter("system_time_raw");
//      ros::Time time_after_measurement=ros::Time::now();
//      measurement_duration=(time_after_measurement-time_before_measurement).toSec();

//      unsigned long long int raw_time_long = std::strtoull(raw_lidar_time.c_str(), NULL, 10);
//      unsigned long long int raw_time_long_sec=raw_time_long >> 32;
//      unsigned long long int raw_time_long_fract_sec=raw_time_long & 0xffffffff;
//      double fract_sec= raw_time_long_fract_sec/(std::pow(2,32));

//      ros::Time lidar_time;
//      lidar_time.fromSec(raw_time_long_sec);
//      lidar_time=lidar_time + ros::Duration(fract_sec);
//      ros::Duration lidar_start_time_offset = (time_before_measurement+ros::Duration(measurement_duration/2))-lidar_time;
//      ros::Duration measure_offset=lidar_start_time_offset - ros::Duration(before_sync_time.toSec());
//      offset_nano=offset_nano+measure_offset.toNSec();
//    }
//    offset_nano=offset_nano/number_of_sync_measurements;
//    ros::Duration measure_offset_averaged(offset_nano*0.000000001);
//    lidar_start_time = before_sync_time + measure_offset_averaged;

//    std::cout << "LIDAR start time for SYNC (nSec) :" << std::endl;
//    std::cout << lidar_start_time.toNSec() << std::endl;
//    std::cout << "MAX SYC error(Sec) :" << std::endl;
//    std::cout << measurement_duration/2.0 << std::endl;

    // Start capturing scanner data
    //-------------------------------------------------------------------------
    std::cout << "Starting capturing: ";
    if( driver_->startCapturingUDP(start_angle_,max_num_points_scan_) )
        std::cout << "OK" << std::endl;
    else
    {
        std::cout << "FAILED!" << std::endl;
        return false;
    }

    return true;
}

//-----------------------------------------------------------------------------
ros::Duration R2000Node::getTimeOffset()
{
  // Clock sync procedure
  //-------------------------------------------------------------------------

  double measurement_duration;
  ros::Time time_before_measurement=ros::Time::now();
  std::string raw_lidar_time = driver_->getParameter("system_time_raw");
  ros::Time time_after_measurement=ros::Time::now();
  measurement_duration=(time_after_measurement-time_before_measurement).toSec();

  unsigned long long int raw_time_long = std::strtoull(raw_lidar_time.c_str(), NULL, 10);
  unsigned long long int raw_time_long_sec=raw_time_long >> 32;
  unsigned long long int raw_time_long_fract_sec=raw_time_long & 0xffffffff;
  double fract_sec= raw_time_long_fract_sec/(std::pow(2,32));

  ros::Time lidar_time;
  lidar_time.fromSec(raw_time_long_sec);
  lidar_time=lidar_time + ros::Duration(fract_sec);
  ros::Duration lidar_start_time_offset = (time_before_measurement+ros::Duration(measurement_duration/2))-lidar_time;
  return lidar_start_time_offset;
}

//-----------------------------------------------------------------------------
void R2000Node::getScanData()
{
    if( !driver_->isCapturing() )
    {
        std::cout << "ERROR: Laser range finder disconnected. Exiting" << std::endl;
        std::exit(0);
//        while( !connect() )
//        {
//            std::cout << "ERROR: Reconnect failed. Trying again in 2 seconds..." << std::endl;
//            usleep((2*1000000));
//        }
    }
    //Syncs clock before getting measurement
    lidar_start_time=getTimeOffset();
    auto scandata = driver_->getFullScan();
    if( scandata.amplitude_data.empty() || scandata.distance_data.empty() || scandata.headers.empty() )
        return;

    // Get the measurement time stamp
    //-------------------------------------------------------------------------
    unsigned long long int raw_time_long = scandata.headers[0].timestamp_raw;
    unsigned long long int raw_time_long_sec=raw_time_long >> 32;
    unsigned long long int raw_time_long_fract_sec=raw_time_long & 0xffffffff;
    double fract_sec= raw_time_long_fract_sec/(std::pow(2,32));

    ros::Time measurement_time;
    measurement_time.fromSec(raw_time_long_sec);
    measurement_time=measurement_time + ros::Duration(fract_sec);

//    std::cout << "Measurement stamp in lidar clock: " << std::endl;
//    std::cout << measurement_time.toSec() << std::endl;

    sensor_msgs::LaserScan scanmsg;
    scanmsg.header.frame_id = frame_id_;
    scanmsg.header.stamp = measurement_time + lidar_start_time ;

    scanmsg.angle_min = double(scandata.headers[0].first_angle)/10000.0/180.0*M_PI;
    scanmsg.angle_max = scanmsg.angle_min + double(scandata.distance_data.size())/double(samples_per_scan_)*2*M_PI;
    scanmsg.angle_increment = double(scandata.distance_data.size())/double(samples_per_scan_)*2*M_PI/float(scandata.distance_data.size());
    scanmsg.time_increment = 1/std::atof(driver_->getParametersCached().at("scan_frequency").c_str())/float(scandata.distance_data.size());

    scanmsg.scan_time = 1/std::atof(driver_->getParametersCached().at("scan_frequency").c_str());
    scanmsg.range_min = std::atof(driver_->getParametersCached().at("radial_range_min").c_str());
    scanmsg.range_max = std::atof(driver_->getParametersCached().at("radial_range_max").c_str());

    scanmsg.ranges.resize(scandata.distance_data.size());
    scanmsg.intensities.resize(scandata.amplitude_data.size());
    for( std::size_t i=0; i<scandata.distance_data.size(); i++ )
    {
        scanmsg.ranges[i] = float(scandata.distance_data[i])/1000.0f;
        scanmsg.intensities[i] = scandata.amplitude_data[i];
    }
    scan_publisher_.publish(scanmsg);
}

//-----------------------------------------------------------------------------
void R2000Node::cmdMsgCallback(const std_msgs::StringConstPtr &msg)
{
    const std::string& cmd = msg->data;
    static const std::string set_scan_frequency_cmd("set scan_frequency=");
    static const std::string set_samples_per_scan_cmd("set samples_per_scan=");

    // Setting of scan_frequency
    //-------------------------------------------------------------------------
    if( cmd.substr(0,set_scan_frequency_cmd.size()) == set_scan_frequency_cmd )
    {
        std::string value = cmd.substr(set_scan_frequency_cmd.size());
        int frequency = std::atoi(value.c_str());
        if(frequency>=10 && frequency<=50)
        {
            scan_frequency_ = frequency;
            driver_->setScanFrequency(frequency);
        }
    }

    // Setting of samples_per_scan
    //-------------------------------------------------------------------------
    if( cmd.substr(0,set_samples_per_scan_cmd.size()) == set_samples_per_scan_cmd )
    {
        std::string value = cmd.substr(set_samples_per_scan_cmd.size());
        int samples = std::atoi(value.c_str());
        if(samples>=72 && samples<=25200)
        {
            samples_per_scan_ = samples;
            driver_->setSamplesPerScan(samples);
        }
    }
}

//-----------------------------------------------------------------------------
} // NS

int main(int argc, char **argv)
{
    ros::init(argc, argv, "r2000_node", ros::init_options::AnonymousName);
    new pepperl_fuchs::R2000Node();
    //ros::spin();
    return 0;
}
