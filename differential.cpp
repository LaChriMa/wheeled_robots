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

#include "differential.h"

#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/torquesensor.h>

// Using namespaces
using namespace osg;
using namespace std;
namespace lpzrobots{

  Differential::Differential(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                             const DifferentialConf& conf, const string& name)
    : OdeRobot(odeHandle, osgHandle, name, "2.0"), conf(conf){
  }


  Differential::~Differential(){
  }


  void Differential::placeIntern(const Matrix& pose){
    assert(2. * conf.wheelRadius > conf.bodyHeight);
    /** Moving robot upward such that the wheel are not stuck on the ground */
    Matrix initialPose;
	initialPose = pose * Matrix::translate(Vec3(0, 0, 2*conf.wheelRadius) );
    create(initialPose);
  }


  int Differential::getSensorsIntern( sensor* sensors, int sensornumber) {
	int len=5;
	sensors[0]= leftWheelJoint->getPosition1();
	double rightAngle = rightWheelJoint->getPosition1() + conf.initWheelOrientation;
	if( rightAngle > M_PI ) sensors[1]=rightAngle-2*M_PI;
	//else if( rightAngle < M_PI ) sensors[1]=rightAngle+2*M_PI;
	else sensors[1]=rightAngle;
	//sensors[1]=rightAngle;
	sensors[2]= leftWheelJoint->getPosition1Rate();
	sensors[3]= rightWheelJoint->getPosition1Rate();
	
	double Inertia = 0.6;
//	sensors[4]= sensors[4] + motorVel / Inertia * 0.01;
	sensors[4] = 0;

	return len;
  }


  void Differential::setMotorsIntern( const double* motors, int motornumber ) {
	motorVel = motors[0];
	leftWheelJoint->addForce1( motors[0]*conf.wheelRadius );
	rightWheelJoint->addForce1( motors[1]*conf.wheelRadius );
  }


  void Differential::create(const Matrix& pose) {
    /* Creating body */
    auto body = new Cylinder(conf.bodyRadius, conf.bodyHeight);
    body->init(odeHandle, conf.bodyMass, osgHandle.changeColor(Color(1,1,0)));
    body->setPose(pose);
    objects.push_back(body);

	/** artificial motor */
	//auto cWheel = new Cylinder(0.1, 0.01);
    //cWheel->setTexture("Images/chess.rgb");
    //cWheel->init(odeHandle, conf.wheelMass, osgHandle);
    //Matrix cWheelPose = pose;
    //cWheel->setPose(cWheelPose);
    //objects.push_back(cWheel);
    //auto centerWheelJoint = new HingeJoint(body, cWheel,
    //                                          cWheel->getPosition(),
    //                                          Axis(0, 0, 1) * cWheelPose);
    //centerWheelJoint->init(odeHandle, osgHandle);
    //joints.push_back(centerWheelJoint);
    //auto m = std::make_shared<AngularMotor1Axis>(odeHandle, centerWheelJoint,
    //                                                 conf.wheelMotorPower);
    //m->setBaseName("artificial motor");
    //m->setVelovityFactor(conf.wheelMotorMaxSpeed);
    //addSensor(m);
    //addMotor(m);

    /* Creating the left wheel */
    auto lWheel = new Cylinder(conf.wheelRadius, conf.wheelHeight);
    lWheel->setTexture("Images/chess.rgb");
    lWheel->init(odeHandle, conf.wheelMass, osgHandle);
    Matrix lWheelPose =
      Matrix::rotate(M_PI/2., 0, 1, 0) *
      Matrix::translate(conf.bodyRadius + conf.wheelHeight / 2.0, .0, .0) *
      pose;
    lWheel->setPose(lWheelPose);
    objects.push_back(lWheel);
    leftWheelJoint = new HingeJoint(body, lWheel, lWheel->getPosition(),
                                    Axis(0, 0, 1) * lWheelPose);
    leftWheelJoint->init(odeHandle, osgHandle);
    joints.push_back(leftWheelJoint);

    /** Creating the right wheel */
    auto rWheel = new Cylinder(conf.wheelRadius, conf.wheelHeight);
    rWheel->setTexture("Images/chess.rgb");
    rWheel->init(odeHandle, conf.wheelMass, osgHandle);
    Matrix rWheelPose = Matrix::rotate(M_PI/2.0, 0, 1, 0) *
      Matrix::rotate(conf.initWheelOrientation, 1, 0, 0) *
      Matrix::translate(-(conf.bodyRadius + conf.wheelHeight / 2.0), .0, .0) *
      pose;
    rWheel->setPose(rWheelPose);
    objects.push_back(rWheel);
    rightWheelJoint = new HingeJoint(body, rWheel,
                                              rWheel->getPosition(),
                                              Axis(0, 0, 1) * rWheelPose);
    rightWheelJoint->init(odeHandle, osgHandle);
    joints.push_back(rightWheelJoint);

    /** Motors */
    // Left wheel motor, the OdeHandle, the joint and the maximun
    // power that motor will be used to achieve desired speed
    //auto motor = std::make_shared<AngularMotor1Axis>(odeHandle, leftWheelJoint,
    //                                                 conf.wheelMotorPower);
    //motor->setBaseName("left motor");
    //motor->setVelovityFactor(conf.wheelMotorMaxSpeed);
    //addSensor(motor);
    //addMotor(motor);

    /** Right wheel motor */
    //motor = std::make_shared<AngularMotor1Axis>(odeHandle, rightWheelJoint,
    //                                            conf.wheelMotorPower);
    //motor->setBaseName("right motor");
    //motor->setVelovityFactor(conf.wheelMotorMaxSpeed);
    //addSensor(motor);
    //addMotor(motor);

	/** Support wheels */
	if( conf.supportWheels )
	{
	  double swRadius = conf.wheelRadius/4;

      auto fsWheel = new Cylinder(swRadius, conf.wheelHeight);
      fsWheel->setTexture("Images/chess.rgb");
      fsWheel->init(odeHandle, conf.sWheelMass, osgHandle);
      Matrix fsWheelPose =
        Matrix::rotate(M_PI/2., 0, 1, 0) *
        Matrix::translate(0, conf.bodyRadius-swRadius, -conf.wheelRadius+swRadius) *
        pose;
      fsWheel->setPose(fsWheelPose);
      objects.push_back(fsWheel);
      auto bodyFrontWheelJoint = new HingeJoint(body, fsWheel, fsWheel->getPosition(),
                                                Axis(0, 0, 1) * fsWheelPose);
      bodyFrontWheelJoint->init(odeHandle, osgHandle);
      joints.push_back(bodyFrontWheelJoint);

      auto bsWheel = new Cylinder(swRadius, conf.wheelHeight);
      bsWheel->setTexture("Images/chess.rgb");
      bsWheel->init(odeHandle, conf.sWheelMass, osgHandle);
      Matrix bsWheelPose =
        Matrix::rotate(M_PI/2., 0, 1, 0) *
        Matrix::translate(0, -( conf.bodyRadius-swRadius ), -conf.wheelRadius+swRadius) *
        pose;
      bsWheel->setPose(bsWheelPose);
      objects.push_back(bsWheel);
      auto bodyBackWheelJoint = new HingeJoint(body, bsWheel, bsWheel->getPosition(),
                                                Axis(0, 0, 1) * bsWheelPose);
      bodyBackWheelJoint->init(odeHandle, osgHandle);
      joints.push_back(bodyBackWheelJoint);
	}



	//addSensor(std::make_shared<AxisOrientationSensor>(AxisOrientationSensor::Mode Axis));
	//AxisOrientationSensor::Mode axmode = AxisOrientationSensor::Axis;
    //AxisOrientationSensor* axsens = new AxisOrientationSensor(axmode);
	//axsens->setBaseName("Orientation of left wheel");
	//addSensor(std::shared_ptr<Sensor>(axsens));
	//axsens->init(lWheel, leftWheelJoint);

    //TorqueSensor* tsl = new TorqueSensor(1);
    //tsl->setBaseName("left torque sensor");
    //addSensor(std::shared_ptr<Sensor>(tsl), Attachment(-1,0));
    //tsl->init(lWheel, leftWheelJoint);

	//vector<Joint*> j;
	//j = joints;
	//cout << endl << "number of joint elements: " << j.size() << endl;
	//cout << "joint pos: " << bodyBackWheelJoint->getPosition1() << endl << endl;
	


    /* Infra-red sensors */
    //auto irSensorBank = std::make_shared<RaySensorBank>();
    //// Initialising infra-red sensor bank
    //irSensorBank->setInitData(odeHandle, osgHandle, TRANSM(0,0,0));
    //irSensorBank->setBaseName("IR ");
    //irSensorBank->setNames({"left", "left front", "front left","front right",
    //      "right front", "right","back left", "back right"});

    //// Registering the sensor in the bank (set of ir sensors), fixed to body
    //// For the first sensor it is rotated to point forward
    //// translation from center of body to outside and middle of height
    //// pose is relative to the parent body - no need to multiply by 'pose'.
    //// Maximum range of sensor value.
    //// drawAll will display a line and the sensor body in the rendered scene.
    //irSensorBank->registerSensor(new IRSensor(), body,
    //                             Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
    //                             Matrix::rotate( M_PI / 2.0, 0, 0, 1) *
    //                             Matrix::translate(-conf.bodyRadius * sin(M_PI * .4),
    //                                               conf.bodyRadius * cos(M_PI * .4),
    //                                               conf.bodyHeight / 2.0),
    //                            conf.irRange,
    //                            RaySensor::drawAll);

    //irSensorBank->registerSensor(new IRSensor(), body,
    //                            Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
    //                            Matrix::rotate( M_PI / 3.5, 0, 0, 1) *
    //                            Matrix::translate(-conf.bodyRadius * sin(M_PI * .25),
    //                                              conf.bodyRadius * cos(M_PI * .25),
    //                                              conf.bodyHeight / 2.0),
    //                            conf.irRange,
    //                            RaySensor::drawAll);

    //irSensorBank->registerSensor(new IRSensor(), body,
    //                            Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
    //                            Matrix::translate(-conf.bodyRadius * sin(M_PI * .05),
    //                                              conf.bodyRadius * cos(M_PI * .05),
    //                                              conf.bodyHeight / 2.0),
    //                            conf.irRange,
    //                            RaySensor::drawAll);

    //irSensorBank->registerSensor(new IRSensor(), body,
    //                            Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
    //                            Matrix::translate(conf.bodyRadius * sin(M_PI * .05),
    //                                              conf.bodyRadius * cos(M_PI * .05),
    //                                              conf.bodyHeight / 2.0),
    //                            conf.irRange,
    //                            RaySensor::drawAll);

    //irSensorBank->registerSensor(new IRSensor(), body,
    //                            Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
    //                            Matrix::rotate(-M_PI / 3.5, 0, 0, 1) *
    //                            Matrix::translate(conf.bodyRadius * sin(M_PI * .25),
    //                                              conf.bodyRadius * cos(M_PI * .25),
    //                                              conf.bodyHeight / 2.0),
    //                            conf.irRange,
    //                            RaySensor::drawAll);

    //irSensorBank->registerSensor(new IRSensor(), body,
    //                            Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
    //                            Matrix::rotate(-M_PI / 2.0, 0, 0, 1) *
    //                            Matrix::translate(conf.bodyRadius * sin(M_PI * .4),
    //                                              conf.bodyRadius * cos(M_PI * .4),
    //                                              conf.bodyHeight / 2.0),
    //                            conf.irRange,
    //                            RaySensor::drawAll);

    //irSensorBank->registerSensor(new IRSensor(), body,
    //                            Matrix::rotate(M_PI / 2.0, 1, 0, 0) *
    //                            Matrix::translate(-conf.bodyRadius * sin(M_PI * .9),
    //                                              conf.bodyRadius * cos(M_PI * .9),
    //                                              conf.bodyHeight / 2.0),
    //                            conf.irRange,
    //                            RaySensor::drawAll);

    //irSensorBank->registerSensor(new IRSensor(), body,
    //                            Matrix::rotate(M_PI / 2.0, 1, 0, 0) *
    //                            Matrix::translate(conf.bodyRadius * sin(M_PI * .9),
    //                                              conf.bodyRadius * cos(M_PI * .9),
    //                                              conf.bodyHeight / 2.0),
    //                            conf.irRange,
    //                            RaySensor::drawAll);
    //addSensor(irSensorBank);

  }
}
