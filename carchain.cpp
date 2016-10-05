/***************************************************************************
 *   Copyright (C) 2005-2013 LpzRobots development team                    *
 *   Authors:                                                              *
 *    Simón Smith <artificialsimon at ed dot ac dot uk>                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/

#include "carchain.h"

#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/torquesensor.h>

// Using namespaces
using namespace osg;
using namespace std;
namespace lpzrobots{

  CarChain::CarChain(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                             const CarChainConf& conf, const string& name)
    : OdeRobot(odeHandle, osgHandle, name, "2.0"), conf(conf) {
   
   sensorNo=2; // nur temporär 
  }


  CarChain::~CarChain() {
  }


  void CarChain::placeIntern(const Matrix& pose) {
    //assert(2. * conf.wheelRadius > conf.bodyHeight);
    /** Moving robot upward such that the wheel are not stuck on the ground */
    Matrix initialPose;
	initialPose = pose * Matrix::translate(Vec3(0, 0, 2*conf.wheelRadius) );
    create(initialPose);
  }


  int CarChain::getSensorsIntern( sensor* sensors, int sensornumber) {
	//sensors[0]= leftWheelJoint->getPosition1();
	//double rightAngle = rightWheelJoint->getPosition1() + conf.initWheelOrientation;
	//if( rightAngle > M_PI ) sensors[1]=rightAngle-2*M_PI;
	////else if( rightAngle < M_PI ) sensors[1]=rightAngle+2*M_PI;
	//else sensors[1]=rightAngle;
	////sensors[1]=rightAngle;
	//sensors[2]= leftWheelJoint->getPosition1Rate();
	//sensors[3]= rightWheelJoint->getPosition1Rate();
	//
	//double Inertia = 0.6;
//	//sensors[4]= sensors[4] + motorVel / Inertia * 0.01;
	//sensors[4] = 0;


    sensors[0] = dynamic_cast<HingeJoint*>(joints[0])->getPosition1();
    sensors[1] = dynamic_cast<HingeJoint*>(joints[1])->getPosition1();
    int len = sensorNo;
	return len;
  }


  void CarChain::setMotorsIntern( const double* motors, int motornumber ) {
      dynamic_cast<HingeJoint*>(joints[0])->addForce1( 1.5 );
      dynamic_cast<HingeJoint*>(joints[1])->addForce1( 1. );
  }


  void CarChain::create(const Matrix& pose) {

    vector<Cylinder*> bodies;        /** bodies for the cars */
    bodies.resize( conf.carNumber );
    vector<Cylinder*> wheels;        /** wheels of the cars */
    wheels.resize( 4*conf.carNumber );
    vector<HingeJoint*> wheelJoints; /** joints between wheels and bodies */
    wheelJoints.resize( 4*conf.carNumber );
    vector<Joint*> carJoints;        /** connections between cars */
    carJoints.resize( conf.carNumber-1 );
   
    for( int i=0; i<conf.carNumber; i++) {
        /** Creating body */
        bodies[i] = new Cylinder( conf.bodyRadius, conf.bodyHeight );
        //bodies[i]->setTexture("Images/chess.rgb");
        bodies[i]->init( odeHandle, conf.bodyMass, osgHandle.changeColor(Color(1,1,0)) );
        Matrix bodyPos = Matrix::translate(0,-i*conf.carDistance*conf.bodyRadius,0)*pose;
        bodies[i]->setPose( bodyPos );
        objects.push_back(bodies[i]);
        /** Creating Left Wheels */
        wheels[2*i] = new Cylinder( conf.wheelRadius, conf.wheelHeight );
        wheels[2*i]->setTexture("Images/chess.rgb");
        wheels[2*i]->init( odeHandle, conf.wheelMass, osgHandle );
        Matrix lwPos = Matrix::rotate(M_PI/2.,0.,1.,0.)*
                       Matrix::translate( conf.bodyRadius+conf.wheelHeight/2.,0.,0.)*
                       bodyPos;
        wheels[2*i]->setPose( lwPos );
        objects.push_back( wheels[2*i] );
        /** Creating Wheel Joints */
        wheelJoints[2*i] = new HingeJoint( bodies[i], wheels[2*i], wheels[2*i]->getPosition(), Axis(0,0,1)*lwPos );
        wheelJoints[2*i]->init( odeHandle, osgHandle );
        joints.push_back( wheelJoints[2*i] );
        motorNo++;
        /** Creating Right Wheels */
        wheels[2*i+1] = new Cylinder( conf.wheelRadius, conf.wheelHeight );
        wheels[2*i+1]->setTexture("Images/chess.rgb");
        wheels[2*i+1]->init( odeHandle, conf.wheelMass, osgHandle );
        Matrix rwPos = Matrix::rotate(M_PI/2.,0.,1.,0.)*
                       Matrix::translate( -(conf.bodyRadius+conf.wheelHeight/2.),0.,0.)*
                       bodyPos;
        wheels[2*i+1]->setPose( rwPos );
        objects.push_back( wheels[2*i+1] );
        /** Creating Wheel Joints */
        wheelJoints[2*i+1] = new HingeJoint( bodies[i], wheels[2*i+1], wheels[2*i+1]->getPosition(), Axis(0,0,1)*rwPos );
        wheelJoints[2*i+1]->init( odeHandle, osgHandle );
        joints.push_back( wheelJoints[2*i+1] );
        motorNo++;
    }

	//if( conf.supportWheels )
	//{ /** Spherical support wheels with ball joints */
	//  double swRadius = conf.wheelRadius/4;
	//  double yPoseSW = -conf.wheelRadius+swRadius;

	//  auto fsWheel = new Sphere(swRadius);
	//  fsWheel->init( odeHandle, conf.sWheelMass, osgHandle);
	//  Matrix fsWheelPose = Matrix::translate(0, conf.bodyRadius-2*swRadius, yPoseSW)*pose;
	//  fsWheel->setPose( fsWheelPose );
	//  objects.push_back( fsWheel );
    //  auto bodyFrontWheelJoint = new BallJoint( body, fsWheel, fsWheel->getPosition());
    //  bodyFrontWheelJoint->init(odeHandle, osgHandle, true, conf.wheelRadius/8. );
    //  joints.push_back(bodyFrontWheelJoint);

	//  auto bsWheel = new Sphere(swRadius);
	//  bsWheel->init( odeHandle, conf.sWheelMass, osgHandle);
	//  Matrix bsWheelPose = 	Matrix::translate(0, -conf.bodyRadius+2.*swRadius, yPoseSW) *pose;
	//  bsWheel->setPose( bsWheelPose );
	//  objects.push_back( bsWheel );
    //  auto bodyBackWheelJoint = new BallJoint(body, bsWheel, bsWheel->getPosition());
    //  bodyBackWheelJoint->init(odeHandle, osgHandle, true, conf.wheelRadius/8. );
    //  joints.push_back(bodyBackWheelJoint);
	//}
 
    for( int i=0; i<conf.carNumber-1; i++) {
        //Matrix frontBodyPos = Matrix::translate(-i*conf.carDistance*conf.bodyRadius,0,0)*pose ;
        //Matrix jointPos = Matrix::translate(-conf.carDistance/2.,0.,0.) *frontBodyPos;
        //Vec3 jointPos = Vec3(0., -conf.carDistance/2.-i*conf.carDistance*conf.bodyRadius, conf.bodyHeight/2.);
        Vec3 jointPos = Vec3(0., -conf.carDistance/2.-i*conf.carDistance*conf.bodyRadius, 0.)*pose;
        carJoints[i] = new BallJoint( bodies[i], bodies[i+1], jointPos );
        carJoints[i]->init( odeHandle, osgHandle, true, conf.bodyHeight/2.);
        joints.push_back( carJoints[i] );
    }

  }
}
