rostopic echo -b 2020-06-29-10-27-01.bag -p /drone1/raw_imu > drone1_imu.csv
rostopic echo -b 2020-06-29-10-27-01.bag -p /drone2/raw_imu > drone2_imu.csv
rostopic echo -b 2020-06-29-10-27-01.bag -p /drone1/odom > drone1_gps_noise.csv
rostopic echo -b 2020-06-29-10-27-01.bag -p /drone2/odom > drone2_gps_noise.csv
rostopic echo -b 2020-06-29-10-27-01.bag -p /drone2/odom_real > drone2_gps_noiseless.csv
rostopic echo -b 2020-06-29-10-27-01.bag -p /drone1/odom_real > drone1_gps_noiseless.csv