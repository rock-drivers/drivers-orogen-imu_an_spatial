/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <rtt/extras/FileDescriptorActivity.hpp>
#include <math.h>
#include <imu_an_spatial/rs232/rs232.h>
#include <base-logging/Logging.hpp>
#include <base/Time.hpp>
#include <iostream>
#include <Eigen/Core>
#include <GeographicLib/Geocentric.hpp>

#define RADIANS_TO_DEGREES (180.0/M_PI)

using namespace imu_an_spatial;
using namespace GeographicLib;

int an_packet_transmit(an_packet_t *an_packet)
{
    an_packet_encode(an_packet);
    return SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
}

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

bool Task::configureHook()
{

    if(OpenComport(const_cast<char*>(_port.get().c_str()), _baudrate.get())){
        LOG_ERROR_S << "Could not open port." << std::endl;
        return false;
    }
    fd = getFileDescriptor();
    an_decoder_initialise(&an_decoder);

    Geocentric earth(Constants::WGS84_a(),Constants::WGS84_f());
    base::Vector3d origin = _local_cartesian_origin.get();
    lc = new LocalCartesian(origin[0],origin[1],origin[2],earth);

    if (! TaskBase::configureHook())
        return false;
    return true;
}
bool Task::startHook()
{
    RTT::extras::FileDescriptorActivity* activity =
    getActivity<RTT::extras::FileDescriptorActivity>();

    if (activity) activity->watch(fd);

    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    RTT::extras::FileDescriptorActivity* fd_activity =
            getActivity<RTT::extras::FileDescriptorActivity>();

    base::samples::IMUSensors imu_sample;
    base::samples::RigidBodyState imu_pose;
    gps_base::Solution sol;
    base::samples::RigidBodyState external_velocity;
    gps_base::Errors errors;

    
    if (fd_activity)
    {
        if (fd_activity->hasError())
        {
            //TODO handle fd error
        }
        else if (fd_activity->hasTimeout())
        {
            //TODO handle timeout
        }
        else
        {
            while(_external_velocity_in.read(external_velocity) == RTT::NewData){
                an_packet_t* an_packet;
                external_body_velocity_packet_t body_vel_packet;
                memset(&body_vel_packet, 0, sizeof(external_body_velocity_packet_t));
                body_vel_packet.velocity[0] = external_velocity.velocity.x();
                body_vel_packet.velocity[1] = external_velocity.velocity.y();
                body_vel_packet.velocity[2] = external_velocity.velocity.z();
                body_vel_packet.standard_deviation = _external_velocity_std_dev.get();

                an_packet = encode_external_body_velocity_packet(&body_vel_packet);
                an_packet_transmit(an_packet);
                an_packet_free(&an_packet);
            }
            if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
            {
                /* increment the decode buffer length by the number of bytes received */
                an_decoder_increment(&an_decoder, bytes_received);

                /* decode all the packets in the buffer */
                while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
                {
                    if (an_packet->id == packet_id_system_state) /* system state packet */
                    {
                        /* copy all the binary data into the typedef struct for the packet */
                        /* this allows easy access to all the different values             */
                        if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
                        {
                            imu_pose.time = sol.time = base::Time::now();
                            imu_pose.sourceFrame = _sourceFrame.get();
                            imu_pose.targetFrame = _targetFrame.get();

                            //create rotation matrix to convert NED to NWU
                            Eigen::Matrix3d ned2nwu;
                            ned2nwu << 1, 0 , 0,
                                       0, -1, 0,
                                       0, 0, -1;

                            //base::Vector3d ori_ned(system_state_packet.orientation[0], system_state_packet.orientation[1], system_state_packet.orientation[2]);
                            //base::Vector3d ori = _NED2NWU ? ned2nwu * ori_ned : ori_ned;

                            //// construct quaternion from euler angles, Tait-Bryan zxy
                            //base::Orientation qx,qy,qz;
                            //qx = Eigen::AngleAxisd(ori[0], Eigen::Vector3d::UnitX());
                            //qy = Eigen::AngleAxisd(ori[1], Eigen::Vector3d::UnitY());
                            //qz = Eigen::AngleAxisd(ori[2], Eigen::Vector3d::UnitZ());
                            //imu_pose.orientation =  qx * qy * qz; 
                            imu_pose.orientation = ori;

                            // convert velocity from NED to NWU, if required by property NED2NWU
                            base::Vector3d vel_ned = base::Vector3d(system_state_packet.velocity[0],system_state_packet.velocity[1],system_state_packet.velocity[2]);
                            imu_pose.velocity = _NED2NWU ? ned2nwu * vel_ned : vel_ned;

                            // convert angular velocity from NED to NWU, if required by property NED2NWU
                            base::Vector3d angular_velocity_ned = base::Vector3d(system_state_packet.angular_velocity[0],system_state_packet.angular_velocity[1],system_state_packet.angular_velocity[2]);
                            imu_pose.angular_velocity = _NED2NWU ? ned2nwu * angular_velocity_ned : angular_velocity_ned;

                            //TODO check NWU NED stuff, right now using dedicated quaternion packet from IMU (see below)
                            lat = system_state_packet.latitude * RADIANS_TO_DEGREES;
                            lon = system_state_packet.longitude * RADIANS_TO_DEGREES;
                            height = system_state_packet.height;

                            double x,y,z;
                            lc->Forward(lat,lon,height,x,y,z);
                            imu_pose.position = base::Vector3d(x,y,z);
                            _imu_pose.write(imu_pose);
                            
                            LOG_INFO("System State Packet:\n");
                            LOG_INFO("\tLatitude = %f, Longitude = %f, Height = %f\n", system_state_packet.latitude * RADIANS_TO_DEGREES, system_state_packet.longitude * RADIANS_TO_DEGREES, system_state_packet.height);
                            LOG_INFO("\tRoll = %f, Pitch = %f, Heading = %f\n", system_state_packet.orientation[0] * RADIANS_TO_DEGREES, system_state_packet.orientation[1] * RADIANS_TO_DEGREES, system_state_packet.orientation[2] * RADIANS_TO_DEGREES);


                            gnss_fix_type_e gnss_fix_type = static_cast<gnss_fix_type_e>(system_state_packet.filter_status.b.gnss_fix_type);
                            switch(gnss_fix_type){
                                case gnss_fix_none: sol.positionType = gps_base::NO_SOLUTION; break;
                                case gnss_fix_2d: sol.positionType = gps_base::AUTONOMOUS_2D; break;
                                case gnss_fix_3d: sol.positionType = gps_base::AUTONOMOUS; break;
                                case gnss_fix_sbas: sol.positionType = gps_base::DIFFERENTIAL; break;
                                case gnss_fix_differential: sol.positionType = gps_base::DIFFERENTIAL; break;
                                case gnss_fix_rtk_float: sol.positionType = gps_base::RTK_FLOAT; break;
                                case gnss_fix_rtk_fixed: sol.positionType = gps_base::RTK_FIXED; break;
                                case gnss_fix_omnistar: sol.positionType = gps_base::OMNISTAR; break;
                                default: sol.positionType = gps_base::INVALID; break;
                            }

                            sol.latitude = system_state_packet.latitude * RADIANS_TO_DEGREES;
                            sol.longitude = system_state_packet.longitude * RADIANS_TO_DEGREES;
                            LOG_INFO("SYS STATE PACKET LAT/LON",system_state_packet.latitude,system_state_packet.longitude);
                            sol.altitude = system_state_packet.height;
                            sol.deviationLatitude = system_state_packet.standard_deviation[0] * system_state_packet.standard_deviation[0];
                            sol.deviationLongitude = system_state_packet.standard_deviation[1] * system_state_packet.standard_deviation[1];
                            sol.deviationAltitude = system_state_packet.standard_deviation[2] * system_state_packet.standard_deviation[2];
                            _gps_solution.write(sol);
                        }
                    }
                    else if (an_packet->id == packet_id_quaternion_orientation)
                    {
                        if(decode_quaternion_orientation_packet(&quaternion_orientation_packet, an_packet) == 0)
                        {
                            //TODO use info from state packet and quaternion packet in same port msg, fix times
                           
                            imu_pose.time = base::Time::now();
                            imu_pose.sourceFrame = _sourceFrame.get();
                            imu_pose.targetFrame = _targetFrame.get();

                            //LOG_WARN("Quat packet: [%8.2f,%8.2f,%8.2f,%8.2f]",quaternion_orientation_packet.orientation[0],
                            //        quaternion_orientation_packet.orientation[1], 
                            //        quaternion_orientation_packet.orientation[2],
                            //        quaternion_orientation_packet.orientation[3]);

                            base::Orientation q = base::Quaterniond(quaternion_orientation_packet.orientation[0],
                                                quaternion_orientation_packet.orientation[1],
                                                quaternion_orientation_packet.orientation[2],
                                                quaternion_orientation_packet.orientation[3]);

                            //TODO Definetly check if this does what it should (NED2NWU)
                            Eigen::Quaternion<double, Eigen::DontAlign> ned2nwu_q;
                            ned2nwu_q = base::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
                            if(_NED2NWU) ori = ned2nwu_q * q;
                            else ori = q; 
                            imu_pose.orientation = ori;

                            double x,y,z;
                            lc->Forward(lat,lon,height,x,y,z);
                            imu_pose.position = base::Vector3d(x,y,z);
                            _imu_pose.write(imu_pose);
                           
                        }
                    }

                    else if (an_packet->id == packet_id_raw_sensors) /* raw sensors packet */
                    {
                        if(decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0)
                        {
                            base::Vector3d acc, gyro, mag;

                            imu_sample.time = base::Time::now();
                            
                            acc = base::Vector3d(raw_sensors_packet.accelerometers[0], raw_sensors_packet.accelerometers[1], raw_sensors_packet.accelerometers[2]);
                            gyro = base::Vector3d(raw_sensors_packet.gyroscopes[0], raw_sensors_packet.gyroscopes[1], raw_sensors_packet.gyroscopes[2]);
                            mag = base::Vector3d(raw_sensors_packet.magnetometers[0], raw_sensors_packet.magnetometers[1], raw_sensors_packet.magnetometers[2]);

                            //change axis convention depending on property(NED to NWU)
                            Eigen::Matrix3d ned2nwu;
                            ned2nwu << 1, 0 , 0,
                                       0, -1, 0,
                                       0, 0, -1;
                            imu_sample.acc = _NED2NWU ? ned2nwu * acc : acc; 
                            imu_sample.gyro = _NED2NWU ? ned2nwu * gyro : gyro;
                            imu_sample.mag = _NED2NWU ? ned2nwu * mag : mag; 
                            _imu_samples.write(imu_sample);

                            LOG_INFO("Raw Sensors Packet:\n");
                            LOG_INFO("\tAccelerometers X: %f Y: %f Z: %f\n", raw_sensors_packet.accelerometers[0], raw_sensors_packet.accelerometers[1], raw_sensors_packet.accelerometers[2]);
                            LOG_INFO("\tGyroscopes X: %f Y: %f Z: %f\n", raw_sensors_packet.gyroscopes[0] * RADIANS_TO_DEGREES, raw_sensors_packet.gyroscopes[1] * RADIANS_TO_DEGREES, raw_sensors_packet.gyroscopes[2] * RADIANS_TO_DEGREES);
                        }
                    }
                    else
                    {
                        LOG_INFO("Packet ID %u of Length %u\n", an_packet->id, an_packet->length);
                    }

                    /* Ensure that you free the an_packet when your done with it or you will leak memory */
                    an_packet_free(&an_packet);
                }
            }

        }
    }
    
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    RTT::extras::FileDescriptorActivity* activity =
        getActivity<RTT::extras::FileDescriptorActivity>();
    if (activity)
        activity->clearAllWatches();

    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
