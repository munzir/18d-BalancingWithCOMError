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

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>

#include <krang-utils/balance.hpp>
#include <krang-utils/file_ops.hpp>
#include <krang-utils/convert_pose_formats.hpp>

#include "MyWindow.hpp"

using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;

// Function Prototypes
SkeletonPtr createKrang(string fullRobotPath, string robotName);
SkeletonPtr createFloor(string floorName);

// Main Method
int main(int argc, char* argv[]) {
    // INPUT on below line (input pose filename)
    string inputPosesFilename = "../defaultInit.txt";

    // INPUT on below line (absolute path of robot)
    string fullRobotPath = "/usr/local/share/krang/urdf/Krang/KrangBaseSlopedCollision.urdf";

    // INPUT on below line (robot name)
    string robotName = "krang";

    // INPUT on below line (floor name)
    string floorName = "floor";

    // Read input file
    Eigen::MatrixXd inputPoses;
    try {
        cout << "Reading input poses ...\n";
        inputPoses = readInputFileAsMatrix(inputPosesFilename);
        cout << "|-> Done\n";
    } catch (exception& e) {
        cout << e.what() << endl;
        return EXIT_FAILURE;
    }

    // create and initialize the world
    WorldPtr world(new World);
    assert(world != nullptr);

    // load skeletons
    SkeletonPtr floor = createFloor(floorName);
    SkeletonPtr robot = createKrang(fullRobotPath, robotName);
    //Eigen::MatrixXd balancedPose = angleBalancePose(robot, inputPoses.row(0).transpose());

    //robot->setPositions(munzirToDart(balancedPose.transpose()));
    robot->setPositions(munzirToDart(inputPoses.row(0)));

    world->addSkeleton(floor); //add ground and robot to the world pointer
    world->addSkeleton(robot);

    // create and initialize the world
    //Eigen::Vector3d gravity(0.0,  -9.81, 0.0);
    //world->setGravity(gravity);
    world->setTimeStep(1.0/1000);

    // create a window and link it to the world
    MyWindow window(new Controller(robot));
    window.setWorld(world);

    glutInit(&argc, argv);
    window.initWindow(960, 720, "Forward Simulation");
    glutMainLoop();

    return 0;
}

// Create Krang
SkeletonPtr createKrang(string fullRobotPath, string robotName) {
    // Instantiate krang
    DartLoader loader;
    SkeletonPtr krang = loader.parseSkeleton(fullRobotPath);
    krang->setName(robotName);

    return krang;
}

// Create Floor
SkeletonPtr createFloor(string floorName) {
    SkeletonPtr floor = Skeleton::create(floorName);

    // Give the floor a body
    BodyNodePtr body = floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
    // body->setFrictionCoeff(1e16);

    // Give the body a shape
    double floor_width = 50;
    double floor_height = 0.05;
    std::shared_ptr<BoxShape> box(
        new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
    auto shapeNode
        = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
    shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

    // Put the body into position
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
    body->getParentJoint()->setTransformFromParentBodyNode(tf);

    return floor;
}
