/*
 * An example showing how to generate joint-space motions at incremental perturbation levels.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <random>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"
#include <Eigen/Dense>
#include <unistd.h>

// Function to multiply two rotation matrices in SO(3)
Eigen::Matrix3d multiplyRotationMatrices(const Eigen::Matrix3d& rotation1, const Eigen::Matrix3d& rotation2) {
    return rotation1 * rotation2;
}

Eigen::Matrix3d eulerToRotationMatrix(double roll, double pitch, double yaw) {
    Eigen::Matrix3d rotation;
    
    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);
    
    rotation(0, 0) = cp * cy;
    rotation(0, 1) = sr * sp * cy - cr * sy;
    rotation(0, 2) = cr * sp * cy + sr * sy;
    rotation(1, 0) = cp * sy;
    rotation(1, 1) = sr * sp * sy + cr * cy;
    rotation(1, 2) = cr * sp * sy - sr * cy;
    rotation(2, 0) = -sp;
    rotation(2, 1) = sr * cp;
    rotation(2, 2) = cr * cp;
    
    return rotation;
}

// Function to convert the upper-left corner of a column-major transformation matrix to Eigen::Matrix3d
Eigen::Matrix3d convertUpperLeftToMatrix3d(const std::array<double, 16>& transformation) {
    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> map(transformation.data());
    Eigen::Matrix3d matrix3d = map.block<3, 3>(0, 0);

    return matrix3d;
}

// Function to write Eigen::Matrix3d to the upper-left corner of a column-major transformation matrix
void writeMatrix3dToUpperLeft(const Eigen::Matrix3d& matNew, std::array<double, 16>& T) {
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> map(T.data());
    map.block<3, 3>(0, 0) = matNew;
}

// Function to sample points within a ball
std::vector<std::array<double, 3>> samplePointsInBall(double R, int numPoints) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(-1.0, 1.0);
    
    std::vector<std::array<double, 3>> points;
    for (int i = 0; i < numPoints; ++i) {
        double u = dis(gen);
        double v = dis(gen);
        double w = dis(gen);
        double scale = std::cbrt(u * u + v * v + w * w);
        double r = R * std::abs(dis(gen));

        std::array<double, 3> point;
        point[0] = r * u / scale;
        point[1] = r * v / scale;
        point[2] = r * w / scale;
        
        points.push_back(point);
    }
    
    return points;
}

// Function to sample euler angles
std::vector<std::array<double, 3>> sampleEulerAngle(double limit, int numPoints) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis(-1.0, 1.0);
  
  std::vector<std::array<double, 3>> angles;
  for (int i = 0; i < numPoints; ++i) {
      double u = dis(gen);
      double v = dis(gen);
      double w = dis(gen);

      std::array<double, 3> angle;
      angle[0] = limit * u;
      angle[1] = limit * v;
      angle[2] = limit * w;
      
      angles.push_back(angle);
  }
  
  return angles;
}

// Function to cast euler angles to rotation matrix
void getNewEETransform(std::array<double, 16>& initT, std::array<double, 3> eulerAngles, double scale) {
  double roll = eulerAngles[0] * scale;
  double pitch = eulerAngles[1] * scale;
  double yaw = eulerAngles[2] * scale;

  Eigen::Matrix3d matDelta = eulerToRotationMatrix(roll, pitch, yaw);
  Eigen::Matrix3d matInit = convertUpperLeftToMatrix3d(initT);
  Eigen::Matrix3d matNew = multiplyRotationMatrices(matInit, matDelta);

  // Write new matrix to the transformation
  writeMatrix3dToUpperLeft(matNew, initT);
}

// Function to generate a random pose of the arm given initial configuration (q_init) and a perturbation level (perturbLimit)
std::array<double, 7> generate_random_pose(std::array<double, 7> q_init, double perturbLimit) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis(-M_PI_4, M_PI_4);
  
  std::array<double, 7> q_random;
  for (int i = 0; i < 7; ++i) {
      double u = dis(gen);
      if (i==0 || i==3)
        q_random[i] = q_init[i];
      else
        q_random[i] = q_init[i] + perturbLimit * u;
  }
  
  return q_random;
}


int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_init = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_init);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    sleep(2.0);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    
    // Joint space random shaking
    double perturbLimit = .41;
    double maxPerturbLimit = 1.5;
    bool return_to_init = 0;
    double maxTimes = 4;
    while (perturbLimit <= maxPerturbLimit) {
      std::cout << "Current perturbation limits: " << perturbLimit << std::endl;
      double count = 0;
      while (count < 2*maxTimes) {
        if (return_to_init == 1) {
          MotionGenerator motion_generator(.5, q_init);
          robot.control(motion_generator);
          return_to_init = 0;
        }
        else {
          std::array<double, 7> q_random = generate_random_pose(q_init, perturbLimit);
          MotionGenerator motion_generator(.5, q_random);
          robot.control(motion_generator);
          return_to_init = 1;
        }
        count += 1;
      }
      sleep(1.5); // pause before increasing the perturbation level
      perturbLimit += 0.03;
    }
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
