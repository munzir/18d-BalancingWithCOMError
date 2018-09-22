/*
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_
#define EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_

#include <dart/dart.hpp>
#include <boost/circular_buffer.hpp>
#include <config4cpp/Configuration.h>

#include "../18h-Util/lqr.hpp"
#include "../18h-Util/convert_pose_formats.hpp"
#include "../18h-Util/random.hpp"
#include "../18h-Util/adrc.hpp"

using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace std;
using namespace config4cpp;

class filter {
  public:
    filter(const int dim, const int n)
    {
      samples.set_capacity(n);
      total = Eigen::VectorXd::Zero(dim,1);
    }
    void AddSample(Eigen::VectorXd v)
    {
      if(samples.full())
      {
        total -= samples.front();
      }
      samples.push_back(v);
      total += v;
      average = total/samples.size();
    }

    boost::circular_buffer<Eigen::VectorXd> samples;
    Eigen::VectorXd total;
    Eigen::VectorXd average;

};

/// \brief Operational space controller for 6-dof manipulator
class Controller {
public:
  /// \brief Constructor
  Controller( dart::dynamics::SkeletonPtr _robot);

  /// \brief Destructor
  virtual ~Controller();

//  bool lqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, Eigen::VectorXd& gain);

  void changeRobotParameters(dart::dynamics::SkeletonPtr robot, int bodyParams, double minXCOMError, double maxDeviation, double maxOffset);

  SkeletonPtr qBody1Change(dart::dynamics::SkeletonPtr krang, double change);

  void updatePositions();

  void updateSpeeds();

  Eigen::Vector3d getBodyCOM(dart::dynamics::SkeletonPtr robot);

  /// \brief
  void update(const Eigen::Vector3d& _LeftTargetPosition,const Eigen::Vector3d& _RightTargetPosition);

  /// \brief Get robot
  SkeletonPtr getRobot() const;

  /// \brief Keyboard control
  virtual void keyboard(unsigned char _key, int _x, int _y);

//private:
  /// \brief Robot
  SkeletonPtr mRobot, mGuessRobot;

  /// \brief Control forces
  Eigen::Matrix<double, 18, 1> mForces;

  size_t mSteps;

  Eigen::Matrix<double, 18, 1> mqBodyInit;

  filter *mdqFilt;

  double mRadius, mL;

  Eigen::Matrix<double, 4, 4> mBaseTf;
  Eigen::Matrix<double, 24, 1> mq;
  Eigen::Vector3d mxyz0; // position of frame 0 in the world frame represented in the world frame
  double mpsi;
  double mqBody1;
  Eigen::Matrix<double, 19, 1> mqBody;
  double mthWheel, mthCOM, mthCOM_true;

  Eigen::Matrix<double, 24, 1> mdq;
  Eigen::Vector3d mdxyz0;
  double mdx, mdqBody1, mdpsi;
  Eigen::Matrix<double, 17, 1> mdqBody;
  Eigen::Matrix<double, 19, 1> mdqMin;
  double mdthR, mdthL, mdthWheel, mdthCOM;

  Eigen::Matrix3d mRot0, mdRot0;

  Eigen::Matrix<double, 17, 1> mTauLim;

  bool maxTimeSet = 0;

  double mInitialComAngle;
  World* mVirtualWorld;

  // Observer States
  double mf_thWheel, mf_thCOM;

  // Observer Control
  Eigen::VectorXd mB_thWheel, mB_thCOM;
  Eigen::VectorXd mu_thWheel, mu_thCOM;

  // LQR
  Eigen::MatrixXd mA, mB, mQ, mR;

  // Simulation sampling time
  double mdt;

  // For plotting purposes
  ofstream mOutFile;

  // ESOs
  ESO *mEthWheel;
  ESO *mEthCOM;
};

#endif  // EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_
