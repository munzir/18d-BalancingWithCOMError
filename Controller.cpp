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
Controller::Controller(dart::dynamics::SkeletonPtr _robot)
  : mRobot(_robot)
   {
  assert(_robot != nullptr);
  int dof = mRobot->getNumDofs();
  std::cout << "[controller] DoF: " << dof << std::endl;
  //mForces.setZero(19);
  mForces.setZero(18);
  mSteps = 0;
  mdt = mRobot->getTimeStep();

  // *************** Initialize Guess Robot
  mGuessRobot = mRobot->clone();
  mGuessRobot->setName("GuessRobot");
  mVirtualWorld = new dart::simulation::World;
  mVirtualWorld->addSkeleton(mGuessRobot);
  mGuessRobot->setPositions(mRobot->getPositions());

  int bodyParams = 4; double minXCOMError = 0.02, maxDeviation = 0.50, maxOffset = 0.50;
  changeRobotParameters(mGuessRobot, bodyParams, minXCOMError, maxDeviation, maxOffset);

  // ************** Lock joints
  int joints = mRobot->getNumJoints();
  for(int i=3; i < joints; i++) {
    mRobot->getJoint(i)->setActuatorType(dart::dynamics::Joint::ActuatorType::LOCKED);
    mGuessRobot->getJoint(i)->setActuatorType(dart::dynamics::Joint::ActuatorType::LOCKED);
  }
  //mdqFilt = new filter(25, 100);
  mdqFilt = new filter(24, 100);

  // ************** Wheel Radius and Distance between wheels
  mR = 0.25, mL = 0.68;

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
  //mqBodyInit.tail(17) = qInit.tail(17);
  mqBodyInit.tail(16) = qInit.tail(16);

  //*********************************** Initialize Extended State Observer
  initializeExtendedStateObservers();

  // **************************** Torque Limits
  //mTauLim << 120, 740, 370, 10, 370, 370, 175, 175, 40, 40, 9.5, 370, 370, 175, 175, 40, 40, 9.5;
  //TODO: Which one is the Kinect ahhhhh!!!
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
      std::cout << A(i,j) << ", ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

// ==========================================================================
dart::dynamics::SkeletonPtr Controller::qBody1Change(dart::dynamics::SkeletonPtr robot, double change) {

  // Get qBody1
  Eigen::Matrix<double, 4, 4> baseTf = robot->getBodyNode(0)->getTransform().matrix();
  double psi =  atan2(baseTf(0,0), -baseTf(1,0));
  double qBody1 = atan2(baseTf(0,1)*cos(psi) + baseTf(1,1)*sin(psi), baseTf(2,1));

  // Change qBody1
  qBody1 += change;

  // Convert qBody1 to axis angle
  Eigen::Transform<double, 3, Eigen::Affine> newBaseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
  newBaseTf.prerotate(Eigen::AngleAxisd(-qBody1, Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+psi,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd aa(newBaseTf.matrix().block<3,3>(0,0));

  // Set the axis angle as new robot position
  Eigen::VectorXd q = robot->getPositions();
  q.head(3) = aa.angle()*aa.axis();
  robot->setPositions(q);

  return robot;
}

// ==========================================================================
double fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

// ==========================================================================
void Controller::changeRobotParameters(dart::dynamics::SkeletonPtr robot, int bodyParams, double minXCOMError, double maxDeviation, double maxOffset) {

  dart::dynamics::SkeletonPtr originalRobot = robot->clone();

  int numBodies = robot->getNumBodyNodes();
  dart::dynamics::BodyNodePtr bodyi;
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
Eigen::Vector3d Controller::getBodyCOM(dart::dynamics::SkeletonPtr robot) {
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
  //mqBody.tail(17) = mq.tail(17);
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
  //mdqBody.tail(17) = mdq.tail(17);
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

// ==========================================================================
void Controller::initializeExtendedStateObservers() {

  updatePositions();
  updateSpeeds();

  mthWheel = 0.0;
  mthWheel_hat = 0.0;
  mdthWheel_hat = 0.0;
  mf_thWheel = 0.0;

  mGuessRobot->setPositions(mRobot->getPositions());
  Eigen::Vector3d COM = mRot0*(getBodyCOM(mGuessRobot)-mxyz0);
  mthCOM_hat = atan2(COM(0), COM(2));
  mdthCOM_hat = mdqBody1;
  mf_thCOM = 0.0;
}

// ==========================================================================
void balance_matrix(const Eigen::MatrixXd &A, Eigen::MatrixXd &Aprime, Eigen::MatrixXd &D) {
    // https://arxiv.org/pdf/1401.5766.pdf (Algorithm #3)
    const int p = 2;
    double beta = 2; // Radix base (2?)
    Aprime = A;
    D = Eigen::MatrixXd::Identity(A.rows(), A.cols());
    bool converged = false;
    do {
        converged = true;
        for (Eigen::Index i = 0; i < A.rows(); ++i) {
            double c = Aprime.col(i).lpNorm<p>();
            double r = Aprime.row(i).lpNorm<p>();
            double s = pow(c, p) + pow(r, p);
            double f = 1;
            while (c < r / beta) {
                c *= beta;
                r /= beta;
                f *= beta;
            }
            while (c >= r*beta) {
                c /= beta;
                r *= beta;
                f /= beta;
            }
            if (pow(c, p) + pow(r, p) < 0.95*s) {
                converged = false;
                D(i, i) *= f;
                Aprime.col(i) *= f;
                Aprime.row(i) /= f;
            }
        }
    } while (!converged);
}

// ==========================================================================
// DGEES computes for an N-by-N real nonsymmetric matrix A, the
// eigenvalues, the real Schur form T, and, optionally, the matrix of
// Schur vectors Z.  This gives the Schur factorization A = Z*T*(Z**T).

// Optionally, it also orders the eigenvalues on the diagonal of the
// real Schur form so that selected eigenvalues are at the top left.
// The leading columns of Z then form an orthonormal basis for the
// invariant subspace corresponding to the selected eigenvalues.

// A matrix is in real Schur form if it is upper quasi-triangular with
// 1-by-1 and 2-by-2 blocks. 2-by-2 blocks will be standardized in the
// form
//         [  a  b  ]
//         [  c  a  ]

// where b*c < 0. The eigenvalues of such a block are a +- sqrt(bc).

typedef long int logical; // logical is the return type of select function to be passed to dgees_
typedef logical (selectFcn)(double *, double *); // function pointer type for select function
logical select(double *wr, double *wi) {         // the function to be passed as select function
  return (*wr < 0);
}

// C function declaration for the DGEES fortran function in lapack library
// see link: http://eigen.tuxfamily.org/index.php?title=Lapack 
// for a short tutorial describing how to do this
extern "C" void dgees_(const char* JOBVS, const char* SORT, selectFcn* SELECT, const int* N, double* A, \
    const int* LDA, int* SDIM, double* WR, double* WI, double* VS, const int* LDVS, double* WORK, \
    const int* LWORK, logical* BWORK, int* INFO );

// ==========================================================================
Eigen::Matrix<double, 1, 4> Controller::lqr(Eigen::Matrix<double, 4, 4>& A, Eigen::Matrix<double, 4, 1>& B, Eigen::Matrix<double, 4, 4>& Q, Eigen::Matrix<double, 1, 1>& R) {
  
  Eigen::Matrix<double, 8, 8> H;
  Eigen::MatrixXd M, dM, balM, sMat;
  int n, n2;
  Eigen::Matrix<double, 4, 1> D;
  Eigen::Matrix<double, 8, 1> s;
  Eigen::Matrix<double, 4, 4> X1, X2, L, U, Xa, Xb, X;
  Eigen::Matrix<double, 1, 4> gains;

  // ================== Pre-Schur
  // Hamiltonian
  H << A, -B*R.inverse()*B.transpose(),
       -Q, -A.transpose();

  // State dimension
  n = A.rows();
  n2 = 2*n;

  // balance H
  M = H;
  dM = M.diagonal().asDiagonal();
  balance_matrix(M-dM, balM, sMat);
  for(int i=0; i<8; i++) s(i) = log2(sMat(i,i));
  for(int i=0; i<4; i++) D(i) = round((-s(i) + s(i+4))/2.0); 
  for(int i=0; i<4; i++) {
    s(i) = pow(2.0, D(i));
    s(i+4) = pow(2.0, -D(i));
  }
  D << s(0), s(1), s(2), s(3);
  H = s.asDiagonal()*H*s.asDiagonal().inverse();

  // =================================== Schur
  // Input Arguments for dgees_ call
  int LDA = H.outerStride();
  int LDVS = n2;
  
  // Outputs of dgees_ call
  int SDIM;               // Number of eigenvalues (after sorting) for which SELECT is true
  double WR[n2], WI[n2];  // Real and imaginary parts of the computed eigenvalues in the same order 
                          // that they appear on the diagonal of the output Schur form T
  Eigen::Matrix<double, 8, 8> Z; // Contains the orthogonal matrix Z of Schur vectors
  
  // The fixed part of the scratch space for dgees_ to do calculations
  logical BWORK[n2];

  // Determine the optimal work size (size of the variable-sized scratch space for dgees_ to do its calculations)
  double WORKDUMMY;
  int LWORK = -1;
  int INFO = 0;
  dgees_("V", "S", select, &n2, H.data(), &LDA, &SDIM, &WR[0], &WI[0], Z.data(), &LDVS, \
    &WORKDUMMY, &LWORK, &BWORK[0], &INFO );
  LWORK = int(WORKDUMMY) + 32;
  Eigen::VectorXd WORK(LWORK);

  // dgees call for real schur decomposition along with ordering
  dgees_("V", "S", select, &n2, H.data(), &LDA, &SDIM, &WR[0], &WI[0], Z.data(), &LDVS, \
    WORK.data(), &LWORK, &BWORK[0], &INFO );

  // ======================================== Post-Schur
  // finding solution X to riccati equation 
  X1 = Z.topLeftCorner(4 ,4);
  X2 = Z.bottomLeftCorner(4 ,4);  
  Eigen::PartialPivLU<Eigen::Matrix<double, 4, 4>> lu(X1);
  L = Eigen::Matrix<double, 4, 4>::Identity();
  L.block<4,4>(0,0).triangularView<Eigen::StrictlyLower>() = lu.matrixLU();
  U = lu.matrixLU().triangularView<Eigen::Upper>();
  Xa = ((X2*U.inverse())*L.inverse())*lu.permutationP();
  Xb = (Xa + Xa.transpose())/2.0;
  X = D.asDiagonal()*Xb*D.asDiagonal();

  // Controller gains
  gains = R.inverse()*(B.transpose()*X);

  if(mSteps == 1) {
    cout << "H:" << endl << H << endl << endl;
    cout << "Z:" << endl << Z << endl << endl;
    cout << "L:" << endl << L << endl << endl;
    cout << "U:" << endl << U << endl << endl;
    cout << "Xa:" << endl << Xa << endl << endl;
    cout << "Xb:" << endl << Xb << endl << endl;
    cout << "X:" << endl << X << endl << endl;
    cout << "gains:" << endl << gains << endl << endl;
  }

  return gains;
}


// ==========================================================================
void Controller::updateExtendedStateObserverParameters() {

  // ********************* Extracting Required Parameters from DART URDF

  // Wheeled Inverted Pendulum Parameters (symbols taken from the paper)
  double I_ra = 0;
  double gamma = 1.0;
  double g = 9.81;
  double c_w = 0.1;
  double r_w = mR;
  double m_w;
  double I_wa;
  double M_g;
  double l_g;
  double I_yy;
  double delta, c1, c2; // Intermediate Parameters

  // Our intermediate Variables
  double ixx, iyy, izz, ixy, ixz, iyz;
  Eigen::Vector3d COM;
  int nBodies;
  Eigen::Matrix3d iMat;
  Eigen::Matrix3d iBody;
  Eigen::Matrix3d rot;
  Eigen::Vector3d t;
  Eigen::Matrix3d tMat;
  dart::dynamics::BodyNodePtr b;
  dart::dynamics::Frame* baseFrame;
  double m;

  // Wheel Mass
  m_w = mRobot->getBodyNode("LWheel")->getMass();

  // Wheel inertia (axis)
  mRobot->getBodyNode("LWheel")->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
  I_wa = ixx;

  // Body Mass
  M_g = mRobot->getMass() - 2*m_w;

  // Distance to body COM
  COM = mRot0*(getBodyCOM(mRobot) - mxyz0); COM(1) = 0;
  l_g = COM.norm();

  // Body inertia (axis)
  nBodies = mRobot->getNumBodyNodes();
  iBody = Eigen::Matrix3d::Zero();
  baseFrame = mRobot->getBodyNode("Base");
  for(int i=0; i<nBodies; i++){
    if(i==1 || i==2) continue; // Skip wheels
    b = mRobot->getBodyNode(i);
    b->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
    rot = b->getTransform(baseFrame).rotation();
    t = mRobot->getCOM(baseFrame) - b->getCOM(baseFrame) ; // Position vector from local COM to body COM expressed in base frame
    m = b->getMass();
    iMat << ixx, ixy, ixz, // Inertia tensor of the body around its CoM expressed in body frame
            ixy, iyy, iyz,
            ixz, iyz, izz;
    iMat = rot*iMat*rot.transpose(); // Inertia tensor of the body around its CoM expressed in base frame
    tMat << (t(1)*t(1)+t(2)*t(2)), (-t(0)*t(1)),          (-t(0)*t(2)),
            (-t(0)*t(1)),          (t(0)*t(0)+t(2)*t(2)), (-t(1)*t(2)),
            (-t(0)*t(2)),          (-t(1)*t(2)),          (t(0)*t(0)+t(1)*t(1));
    iMat = iMat + m*tMat; // Parallel Axis Theorem
    iBody += iMat;
  }
  I_yy = iBody(0, 0);

  // Intermediate Parameters
  delta = (M_g*l_g+I_yy+pow(gamma,2)*I_ra)*(M_g+m_w)*pow(r_w,2)+I_wa+I_ra*pow(gamma,2)-pow(M_g*r_w*l_g-I_ra*pow(gamma,2),2);
  c1 = (M_g+m_w)*pow(r_w,2)+I_wa+I_ra*pow(gamma,2)+M_g*r_w*l_g+I_ra*pow(gamma,2);
  c2 = M_g*r_w*l_g+M_g*pow(l_g,2)+I_yy;

  // ******************** Robot dynamics for LQR Gains (Not used)
  Eigen::Matrix<double, 4, 4> A, Q;
  Eigen::Matrix<double, 4, 1> B;
  Eigen::Matrix<double, 1, 1> R; 
  
  A << 0, 0, 1, 0,
       0, 0, 0, 1,
       ((M_g+m_w)*pow(r_w,2)+I_wa+I_ra*pow(gamma,2))*M_g*g*l_g/delta, 0, -c1*c_w/delta, c1*c_w/delta,
       (M_g*r_w*l_g-I_ra*pow(gamma,2))*M_g*g*l_g/delta, 0, c2*c_w/delta, -c2*c_w/delta;

  B << 0,
       0,
       -c1/delta,
       c2/delta;
  if(mSteps == 1) {
    cout << A << endl;
    cout << B << endl;
  }

  A << 0, 0, 1, 0,
       0, 0, 0, 1,
       17.7828531704201,  0, -0.00858417221300891,  0.00858417221300891,
       47.8688365622367,  0, 0.0288387521185202,  -0.0288387521185202;
  B << 0,
       0,
       -0.0858417221300890,
       0.288387521185202;
  Q << 300*1, 0, 0, 0,
       0, 300*320, 0, 0, 
       0, 0, 300*100, 0,
       0, 0, 0, 300*300;
  R << 500;
  mF = lqr(A, B, Q, R);
  
  // ********************** Observer Dynamics
  mA_ << 0, 1, 0,
         0, 0, 1,
         0, 0, 0;
  mB_1 << 0,
         c2/delta,
         0;
  mB_2 << 0,
         -c1/delta,
         0;
  // mB_1 << 0,
  //        0.288387521185202,
  //        0;
  // mB_2 << 0,
  //        -0.0858417221300890,
  //        0;


  // ***************** Observer Feedback Gains
  // mL_1 << 1760.00000000439,
  //        269438.396409193,
  //        21501434.2537036;
  // mL_2 << 1760.00000000439,
  //        269438.396409193,
  //        21501434.2537036;

  mL_1 << 1159.99999999673,173438.396407957,1343839.4084839;
  mL_2 = mL_1;
}

// ==========================================================================
void Controller::updateExtendedStateObserverStates() {

  // Integrate Dynamics
  // mA_, mL_1, mB_1, mL_2, mB_2;
  Eigen::Vector3d x, xdot;

  x << mthWheel_hat, mdthWheel_hat, mf_thWheel;
  xdot = mA_*x + mL_1*(mthWheel - mthWheel_hat) + mB_1*mu_thWheel;
  x += xdot*mdt; mthWheel_hat = x(0); mdthWheel_hat = x(1); mf_thWheel = x(2);

  x << mthCOM_hat, mdthCOM_hat, mf_thCOM;
  xdot = mA_*x + mL_2*(mthCOM - mthCOM_hat) + mB_2*mu_thCOM;
  x += xdot*mdt; mthCOM_hat = x(0); mdthCOM_hat = x(1); mf_thCOM = x(2);
}

// ==========================================================================
double Controller::activeDisturbanceRejectionControl() {

  // ****************** The following F comes from Bodgan and Nathan's code
  // Eigen::Matrix<double, 4, 1> F;
  // F << -444.232838220247,   -13.8564064605507,   -111.681669536162,   -26.3837189119712;
  // F << -1848.03012979190,    -13.8564064605509,   -269.597433286135,   -18.3661533292315;
  // Working at 0 Tilt
  // F << -2443.80123228903,    -21.9089023002056,   -366.198685841371,   -28.8559866982220;
  // F << -500, -20, -400, -30;
  // Break Dance Moves
  // F << -510.449550243191,  -0.244948974282393,  -110.528023245082, -1.14116859943037;
  // F << -502.507542429414, -0.182574185839181,  -109.159318118851, -0.994601287695807;
  // Indefinite Drift
  // F << -504.249727091444,  -0.199999999999294,  -109.459542459413, -1.02733232811016;
  // F << -504.249793519900, -0.200000000000233,  -109.459675610606, -1.02733243214781;
  // Explodes ???
  // F << -505.168127552260,  -0.200000000003292,  -109.618042362170, -1.04301620674174;
  // F << -500.481166823486,  -0.173205080751090,  -108.809840324047, -0.958459823947375;
  // GOOD
  // F << -500.4811784741, -0.1732050808, -108.8098652890, -0.9584598411;
  // F << -3617.30655560822,    -69.2820323027686,   -676.022987559394,   -95.4128736402033;
  // F << -6802.85419804756, -69.2820323027654,   -928.328202601923,   -86.9485793475169;
  // F << -2669.67918242245,   -21.9089023002072,   -371.814613562732,   -28.2110280320200;
  // F << -2549.11511950484,   -21.9089023002060,   -368.094666786882,   -28.5125368751267;
  // F << -2443.80123228903,    0.0,   0.0,   0.0;
  // F << -953.5142, -13.8564, -330.9289, -29.5582; // By Munzir, based on A, B matrices calculated in updateESOParameters() function


  // Observer Control Gains
  double F_thWheel = mF(1);
  double F_dthWheel = mF(3);
  double F_thCOM = mF(0);
  double F_dthCOM = mF(2);

  // Observer Control Update
  mu_thWheel = -F_thWheel*(mthWheel_hat - 0) - F_dthWheel*(mdthWheel_hat - 0);
  mu_thCOM = -F_thCOM*(mthCOM_hat - 0) - F_dthCOM*(mdthCOM_hat - 0);

  // Active Disturbance Rejection Control
  return mu_thWheel + mu_thCOM - mf_thWheel/mB_1(1) - mf_thCOM/mB_2(1);

  // Simple LQR
  // Eigen::VectorXd ut1(2); ut1 << F[1], F[3];
  // Eigen::VectorXd ut2(2); ut2 << xFull[1]-xDesired[1], xFull[3]-xDesired[3];
  // Eigen::VectorXd up1(2); up1 << F[0], F[2];
  // Eigen::VectorXd up2(2); up2 << xFull[0]-xDesired[0], xFull[2]-xDesired[2];
  // u_theta = -ut1.transpose()*ut2;
  // u_psi = -up1.transpose()*up2;
  // tau_w = u_psi + u_theta;
  // tauL = 0.5*tau_w;
  // tauR = 0.5*tau_w;
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

  // Apply the Control
  double wheelsTorque;
  wheelsTorque = activeDisturbanceRejectionControl();
  mForces(0) = 0.5*wheelsTorque;
  mForces(1) = 0.5*wheelsTorque;
  //const vector<size_t > index{6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};
  const vector<size_t > index{6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};
  mRobot->setForces(index, mForces);

  // Update Extended State Observer
  updateExtendedStateObserverParameters();
  updateExtendedStateObserverStates();

  // Dump data
  mOutFile << mthCOM_true << ", " << mthCOM << endl;
}

//=========================================================================
dart::dynamics::SkeletonPtr Controller::getRobot() const {
  return mRobot;
}

//=========================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/) {
}
