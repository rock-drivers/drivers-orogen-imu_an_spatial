#!/usr/bin/ruby

require 'orocos'
include Orocos

Orocos.initialize

Orocos.run 'imu_an_spatial::Task' => 'imu' do
  
  imu = TaskContext.get 'imu'
#imu.port = '/dev/ttyUSB0'
#  imu.port = '/dev/ttyS3'
  imu.port = '/home/leifole/ttyGPS'
  imu.baudrate = 115200
  imu.NED2NWU = false; 
  imu.local_cartesian_origin = Eigen::Vector3.new(38.4129, -110.7836, 1350.4)

  imu.configure
  imu.start

  Orocos.watch(imu)
end
