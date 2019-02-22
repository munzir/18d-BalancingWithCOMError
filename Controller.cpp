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

#include "Controller.hpp"

//==========================================================================
Controller::Controller(SkeletonPtr _robot)
  : mRobot(_robot)
   {
  assert(_robot != nullptr);
  int dof = mRobot->getNumDofs();
  std::cout << "[controller] DoF: " << dof << std::endl;
  mForces.setZero(18);
  mSteps = 0;
  mdt = mRobot->getTimeStep();

  // *************** Initialize Guess Robot
  mGuessRobot = mRobot->cloneSkeleton();
  mGuessRobot->setName("GuessRobot");
  mVirtualWorld = new World;
  mVirtualWorld->addSkeleton(mGuessRobot);
  mGuessRobot->setPositions(mRobot->getPositions());

  int bodyParams = 4; double minXCOMError = 0.02, maxDeviation = 0.50, maxOffset = 0.50;
  changeRobotParameters(mGuessRobot, bodyParams, minXCOMError, maxDeviation, maxOffset);

  // ************** Lock joints
  int joints = mRobot->getNumJoints();
  for(int i=3; i < joints; i++) {
    mRobot->getJoint(i)->setActuatorType(Joint::ActuatorType::LOCKED);
    mGuessRobot->getJoint(i)->setActuatorType(Joint::ActuatorType::LOCKED);
  }
  mdqFilt = new filter(24, 100);

  // ************** Wheel Radius and Distance between wheels
  mRadius = 0.25, mL = 0.68;

  // *********************************** Tunable Parameters
  Configuration *  cfg = Configuration::create();
  const char *     scope = "";
  const char *     configFile = "../controlParams.cfg";

  try {
    cfg->parse(configFile);

    // -- com error
    mInitialComAngle = M_PI*(cfg->lookupFloat(scope, "initialComAngle")/180);
  } catch(const ConfigurationException & ex) {
      cerr << ex.c_str() << endl;
      cfg->destroy();
  }
  cfg->destroy();

  // ********************************** Introduce CoM error
  qBody1Change(mRobot, mInitialComAngle);
  qBody1Change(mGuessRobot, mInitialComAngle);
  Eigen::Vector3d com = mRobot->getCOM() - mRobot->getPositions().segment(3,3);
  cout << "robot com: " << com << endl;
  cout << "robot com angle: " << atan2(com(0), com(2))*180.0/M_PI << endl;
  com = mGuessRobot->getCOM() - mGuessRobot->getPositions().segment(3,3);
  cout << "guess robot com: " << com << endl;
  cout << "guess com angle: " << atan2(com(0), com(2))*180.0/M_PI << endl;

  //*********************************** Read Initial Pose for Pose Regulation
  Eigen::Matrix<double, 24, 1> qInit = mRobot->getPositions();
  mBaseTf = mRobot->getBodyNode(0)->getTransform().matrix();
  double psiInit =  atan2(mBaseTf(0,0), -mBaseTf(1,0));
  double qBody1Init = atan2(mBaseTf(0,1)*cos(psiInit) + mBaseTf(1,1)*sin(psiInit), mBaseTf(2,1));
  mqBodyInit(0) = qBody1Init;
  mqBodyInit.tail(16) = qInit.tail(16);

  //*********************************** Initialize Extended State Observer
  updatePositions();
  updateSpeeds();
  mthWheel = 0.0;

  Eigen::Vector3d EthWheel_Init(0.0, 0.0, 0.0);
  Eigen::Vector3d EthWheel_ObsGains(1159.99999999673, 173438.396407957, 1343839.4084839);
  mEthWheel = (Eso*) new Eso(EthWheel_Init, EthWheel_ObsGains);

  mGuessRobot->setPositions(mRobot->getPositions());
  Eigen::Vector3d COM = mRot0*(getBodyCOM(mGuessRobot)-mxyz0);
  Eigen::Vector3d EthCOM_Init(atan2(COM(0), COM(2)), mdqBody1, 0.0);
  Eigen::Vector3d EthCOM_ObsGains(1159.99999999673, 173438.396407957, 1343839.4084839);
  mEthCOM = (Eso*) new Eso(EthCOM_Init, EthCOM_ObsGains);

  mB_thWheel = Eigen::VectorXd::Zero(3);
  mB_thCOM = Eigen::VectorXd::Zero(3);

  mu_thWheel = Eigen::VectorXd::Zero(1);
  mu_thCOM = Eigen::VectorXd::Zero(1);

  //*********************************** LQR Parameters
  mA = Eigen::MatrixXd::Zero(4, 4);
  mB = Eigen::MatrixXd::Zero(4, 1);
  mQ = Eigen::MatrixXd::Zero(4, 4);
  mR = Eigen::MatrixXd::Zero(1, 1);;
  mQ << 300*1, 0, 0, 0,
       0, 300*320, 0, 0,
       0, 0, 300*100, 0,
       0, 0, 0, 300*300;
  mR << 500;

  // **************************** Torque Limits
  mTauLim << 120, 740, 370, 370, 370, 175, 175, 40, 40, 9.5, 370, 370, 175, 175, 40, 40, 9.5;

  // **************************** Data Output
  mOutFile.open("output.csv");
}

//=========================================================================
Controller::~Controller() {
  mOutFile.close();
}

//=========================================================================
void printMatrix(Eigen::MatrixXd A){
  for(int i=0; i<A.rows(); i++){
    for(int j=0; j<A.cols(); j++){
      cout << A(i,j) << ", ";
    }
    cout << endl;
  }
  cout << endl;
}

// ==========================================================================
SkeletonPtr Controller::qBody1Change(SkeletonPtr robot, double change) {

    Eigen::MatrixXd q = dartToMunzir(robot->getPositions(), robot);

    // Change qBody1
    q(1, 0) += change;

    // Assign new pose to robot
    robot->setPositions(munzirToDart(q.transpose()));

    return robot;
}

// ==========================================================================
void Controller::changeRobotParameters(SkeletonPtr robot, int bodyParams, double minXCOMError, double maxDeviation, double maxOffset) {

  SkeletonPtr originalRobot = robot->cloneSkeleton();

  int numBodies = robot->getNumBodyNodes();
  BodyNodePtr bodyi;
  double mi, pert_mi;
  double mxi, pert_mxi;
  double myi, pert_myi;
  double mzi, pert_mzi;

  // Generate perturbed parameters
  double deviation;
  double offset;
  double xCOM = 0;
  while (abs(xCOM) < minXCOMError) {
      for (int i = 0; i < numBodies; i++) {
          bodyi = originalRobot->getBodyNode(i);
          mi = bodyi->getMass();
          mxi = mi * bodyi->getLocalCOM()(0);
          myi = mi * bodyi->getLocalCOM()(1);
          mzi = mi * bodyi->getLocalCOM()(2);

          pert_mi = mi;

          deviation = fRand(-maxDeviation, maxDeviation);
          offset = fRand(-maxOffset, maxOffset);
          pert_mxi = mxi + deviation * mxi + offset;

          deviation = fRand(-maxDeviation, maxDeviation);
          offset = fRand(-maxOffset, maxOffset);
          pert_myi = myi + deviation * myi + offset;

          deviation = fRand(-maxDeviation, maxDeviation);
          offset = fRand(-maxOffset, maxOffset);
          pert_mzi = mzi + deviation * mzi + offset;

          robot->getBodyNode(i)->setMass(pert_mi);
          robot->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(pert_mxi, pert_myi, pert_mzi)/pert_mi);
      }
      xCOM = robot->getCOM()(0);
      cout << "xCOM: " << xCOM << endl;
  }
}
// ==========================================================================
Eigen::Vector3d Controller::getBodyCOM(SkeletonPtr robot) {
  double fullMass = robot->getMass();
  double wheelMass = robot->getBodyNode("LWheel")->getMass();
  return (fullMass*robot->getCOM() - wheelMass*robot->getBodyNode("LWheel")->getCOM() - wheelMass*robot->getBodyNode("RWheel")->getCOM())/(fullMass - 2*wheelMass);
}

// ==========================================================================
void Controller::updatePositions(){
  mBaseTf = mRobot->getBodyNode(0)->getTransform().matrix();
  mq = mRobot->getPositions();
  mxyz0 = mq.segment(3,3); // position of frame 0 in the world frame represented in the world frame
  mpsi =  atan2(mBaseTf(0,0), -mBaseTf(1,0));
  mqBody1 = atan2(mBaseTf(0,1)*cos(mpsi) + mBaseTf(1,1)*sin(mpsi), mBaseTf(2,1));
  mqBody(0) = mqBody1;
  mqBody.tail(16) = mq.tail(16);
  mRot0 << cos(mpsi), sin(mpsi), 0,
          -sin(mpsi), cos(mpsi), 0,
          0, 0, 1;
  mthWheel += mdthWheel*mdt;
  mGuessRobot->setPositions(mRobot->getPositions());
  Eigen::Vector3d bodyCOM = mRot0*(getBodyCOM(mGuessRobot)-mxyz0);
  mthCOM = atan2(bodyCOM(0), bodyCOM(2));
  bodyCOM = mRot0*(getBodyCOM(mRobot)-mxyz0);
  mthCOM_true = atan2(bodyCOM(0), bodyCOM(2)); // for plotting purposes
}

// ==========================================================================
void Controller::updateSpeeds(){
  mdqFilt->AddSample(mRobot->getVelocities());
  mdq = mdqFilt->average;
  mdxyz0 = mBaseTf.matrix().block<3,3>(0,0)*mdq.segment(3,3); // velocity of frame 0 in the world frame represented in the world frame
  mdx = mdq(4)*sin(mqBody1) - mdq(5)*cos(mqBody1);
  mdqBody1 = -mdq(0);
  mdpsi = (mBaseTf.block<3,3>(0,0) * mdq.head(3))(2);
  mdqBody(0) = mdqBody1;
  mdqBody.tail(16) = mdq.tail(16);
  mdqMin(0) = mdx;
  mdqMin(1) = mdpsi;
  mdqMin.tail(17) = mdqBody;
  mdRot0 << (-sin(mpsi)*mdpsi), (cos(mpsi)*mdpsi), 0,
           (-cos(mpsi)*mdpsi), (-sin(mpsi)*mdpsi), 0,
           0, 0, 0;
  mdthL = mdq(6) + mdqBody1;
  mdthR = mdq(7) + mdqBody1;
  mdthWheel = (mdthL + mdthR)/2;
  mdthCOM = mdqBody1;
}

//=========================================================================
void Controller::update(const Eigen::Vector3d& _LeftTargetPosition,const Eigen::Vector3d& _RightTargetPosition) {

  // increase the step counter
  mSteps++;

  // updates mBaseTf, mq, mxyz0, mpsi, mqBody1, mqBody, mRot0
  // Needs mRobot
  updatePositions();

  // updates mdq, mdxyz0, mdx, mdqBody1, mdpsi, mdqBody, mdqMin, dRot0
  // Needs mRobot, mdqFilt, mBaseTf, mqBody1
  updateSpeeds();

  // compute linearized dynamics
  computeLinearizedDynamics(mRobot, mA, mB, mB_thWheel, mB_thCOM);

  // Apply the Control
  double wheelsTorque;
  Eigen::VectorXd refState = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd state = Eigen::VectorXd::Zero(4);
  state << mthCOM, mdthCOM, mthWheel, mdthWheel;
  activeDisturbanceRejectionControl(mA, mB, mQ, mR, mEthWheel, mEthCOM, mB_thWheel, mB_thCOM, state, refState, mdt, mu_thWheel, mu_thCOM);
  wheelsTorque = mu_thCOM(0) + mu_thWheel(0);
  mForces(0) = 0.5*wheelsTorque;
  mForces(1) = 0.5*wheelsTorque;
  //const vector<size_t > index{6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};
  const vector<size_t > index{6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};
  mRobot->setForces(index, mForces);


  // Dump data
  mOutFile << mthCOM_true << ", " << mthCOM << endl;
}

//=========================================================================
SkeletonPtr Controller::getRobot() const {
  return mRobot;
}

//=========================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/) {
}
