
#include <iostream>

#include <ros/ros.h>

#include "filter_test/filter_test.h"
#include "filter_test/filter.h"


#include <assert.h>

#include <glog/logging.h>


enum StateNames {
  kPosition = 0,
  kVelocity,
  kOrientation
};

enum MeasurementDefinition {
  kMeasPosition = 0,
  kMeasImu
};

int main(int argc, char** argv) {
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  // Announce this program to the ROS master
  ros::init(argc, argv, "filter_test_node");
  // Start the node resource managers (communication, time, etc)
  ros::NodeHandle nh("~");

  std::vector<BlockType> state_types {kVector3, kVector3, kVector2};
  std::vector<int> state_names {kPosition, kVelocity, kOrientation};


  Filter testfilter;
  testfilter.defineState(state_types);

  Eigen::Vector3d test_vec(1,2,3);


  testfilter.initStateValue(kPosition, test_vec);


  testfilter.printState();



  ConstantResidual* test_residual = new ConstantResidual();
  std::vector<int> first_keys {kStatePosition};
  std::vector<int> second_keys {kStatePosition};
  std::vector<int> measurement_keys {kMeasPosition};
  testfilter.addResidual(test_residual, first_keys, second_keys, measurement_keys);



  testfilter.checkResiduals();

  testfilter.printTimeline();


  BlockBase* testbase(new VectorBlock<3>());


  Eigen::MatrixXd test;

  int size=10;

  test.resize(size,size);
  test.setZero();

  int a=1;
  int b=3;


  Eigen::Ref<Eigen::MatrixXd> test2=test.block<3,3>(a,b);

  Eigen::Ref<Eigen::Matrix2d> test3= test2.block<2,2>(0,0);

  test3(1,1) = 1;

  std::cout << test;


  std::cout << "element" << test_vec(state_types.at(0)) << std::endl;

  Eigen::Ref<Eigen::VectorXd> test_vec1(test_vec);

  std::cout << test_vec.size();

  // Stop the node's resources
  ros::shutdown();
  // Exit tranquilly
  return 0;
}
