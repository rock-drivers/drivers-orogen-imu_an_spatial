#!/usr/bin/ruby

require 'orocos'
include Orocos

Orocos.initialize

Orocos.run 'imu_an_spatial::Task' => 'imu' do
  
  imu = TaskContext.get 'imu'
#imu.port = '/dev/ttyUSB0'
  imu.port = '/dev/ttyS3'
  imu.baudrate = 115200
  imu.NED2NWU = true 

  imu.configure
  imu.start

  Orocos.watch(imu)
end
