#!/usr/bin/ruby

require 'orocos'
include Orocos

Orocos.initialize

Orocos.run 'imu_an_spatial::Task' => 'imu' do
  
  imu = TaskContext.get 'imu'
  imu.port = '/dev/ttyUSB0'
  imu.baudrate = 115200

  imu.configure
  imu.start

  Orocos.watch(imu)
end
