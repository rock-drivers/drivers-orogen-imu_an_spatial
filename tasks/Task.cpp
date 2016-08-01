/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <rtt/extras/FileDescriptorActivity.hpp>
#include <math.h>
#include <imu_an_spatial/rs232/rs232.h>
#include <base/logging.h>
#include <base/Time.hpp>
#include <iostream>
#include <Eigen/Core>

#define RADIANS_TO_DEGREES (180.0/M_PI)

using namespace imu_an_spatial;

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
    gps::Solution sol;

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

                            // convert orientation from NED to NWU, if required by property NED2NWU
                            base::Vector3d ori_ned(system_state_packet.orientation[0], system_state_packet.orientation[1], system_state_packet.orientation[2]);
                            base::Vector3d ori = _NED2NWU ? ned2nwu * ori_ned : ori_ned;

                            // construct quaternion from euler angles, Tait-Bryan zxy
                            base::Orientation qx,qy,qz;
                            qx = Eigen::AngleAxisd(ori[0], Eigen::Vector3d::UnitX());
                            qy = Eigen::AngleAxisd(ori[1], Eigen::Vector3d::UnitY());
                            qz = Eigen::AngleAxisd(ori[2], Eigen::Vector3d::UnitZ());
                            imu_pose.orientation =  qx * qy * qz; 

                            // convert velocity from NED to NWU, if required by property NED2NWU
                            base::Vector3d vel_ned = base::Vector3d(system_state_packet.velocity[0],system_state_packet.velocity[1],system_state_packet.velocity[2]);
                            imu_pose.velocity = _NED2NWU ? ned2nwu * vel_ned : vel_ned;

                            // convert angular velocity from NED to NWU, if required by property NED2NWU
                            base::Vector3d angular_velocity_ned = base::Vector3d(system_state_packet.angular_velocity[0],system_state_packet.angular_velocity[1],system_state_packet.angular_velocity[2]);
                            imu_pose.angular_velocity = _NED2NWU ? ned2nwu * angular_velocity_ned : angular_velocity_ned;

                            _imu_pose.write(imu_pose);
                            
                            LOG_INFO("System State Packet:\n");
                            LOG_INFO("\tLatitude = %f, Longitude = %f, Height = %f\n", system_state_packet.latitude * RADIANS_TO_DEGREES, system_state_packet.longitude * RADIANS_TO_DEGREES, system_state_packet.height);
                            LOG_INFO("\tRoll = %f, Pitch = %f, Heading = %f\n", system_state_packet.orientation[0] * RADIANS_TO_DEGREES, system_state_packet.orientation[1] * RADIANS_TO_DEGREES, system_state_packet.orientation[2] * RADIANS_TO_DEGREES);

                            // writing to the gps solution port
                            //TODO lat/long to which UTM? see drivers/orogen/gps and gdal UTM methods
                            //TODO set GNSS fix type from system state packet 

                            gnss_fix_type_e gnss_fix_type = static_cast<gnss_fix_type_e>(system_state_packet.filter_status.b.gnss_fix_type);
                            switch(gnss_fix_type){
                                case gnss_fix_none: sol.positionType = gps::NO_SOLUTION; break;
                                case gnss_fix_2d: sol.positionType = gps::AUTONOMOUS_2D; break;
                                case gnss_fix_3d: sol.positionType = gps::AUTONOMOUS; break;
                                case gnss_fix_sbas: sol.positionType = gps::DIFFERENTIAL; break;
                                case gnss_fix_differential: sol.positionType = gps::DIFFERENTIAL; break;
                                case gnss_fix_rtk_float: sol.positionType = gps::RTK_FLOAT; break;
                                case gnss_fix_rtk_fixed: sol.positionType = gps::RTK_FIXED; break;
                                default: sol.positionType = gps::INVALID; break;
                            }

                            sol.latitude = system_state_packet.latitude;
                            sol.longitude = system_state_packet.longitude;
                            sol.altitude = system_state_packet.height;
                            sol.deviationLatitude = system_state_packet.standard_deviation[0] * system_state_packet.standard_deviation[0];
                            sol.deviationLongitude = system_state_packet.standard_deviation[1] * system_state_packet.standard_deviation[1];
                            sol.deviationAltitude = system_state_packet.standard_deviation[2] * system_state_packet.standard_deviation[2];
                            _gps_solution.write(sol);
                        }
                    }
                    else if (an_packet->id == packet_id_raw_sensors) /* raw sensors packet */
                    {
                        /* copy all the binary data into the typedef struct for the packet */
                        /* this allows easy access to all the different values             */
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
