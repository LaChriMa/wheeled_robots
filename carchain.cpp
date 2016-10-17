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
   
    motorNo = 2*conf.carNumber;  /** each car has two wheels */
    sensorNo = (conf.speedSensors ? 2 : 1) * motorNo; /** each wheel has an angle and an angular velocity sensor */
    //if (conf.spring) motorNo += conf.carNumber - 1; //Springs between the cars are not motors
    cout << "########" << endl << "Variables of the CarChain: " << endl;
    cout << "motorNo: " << motorNo << "   and sensorNo: " << sensorNo << endl;
  }


  CarChain::~CarChain() {}


  void CarChain::placeIntern(const Matrix& pose) {
    //assert(2. * conf.wheelRadius > conf.bodyHeight);
    /** Moving robot upward such that the wheel are not stuck on the ground */
    Matrix initialPose;
	initialPose = pose * Matrix::translate( Vec3(0.,0.,2.*conf.wheelRadius) );
    create(initialPose);
  }


  int CarChain::getSensorsIntern( sensor* sensors, int sensornumber) {
    int len=0;
    int jointsPerCar = conf.supportWheels ? 4:2;
    for( int i=0; i<conf.carNumber; i++ ) {
       sensors[len] = dynamic_cast<HingeJoint*>(joints[jointsPerCar*i+0])->getPosition1();
       len++;
       sensors[len] = dynamic_cast<HingeJoint*>(joints[jointsPerCar*i+1])->getPosition1();
       len++;
    }
    if( conf.speedSensors ) {
      for( int i=0; i<conf.carNumber; i++) {
         sensors[len] = dynamic_cast<HingeJoint*>(joints[jointsPerCar*i+0])->getPosition1Rate();
         len++;
         sensors[len] = dynamic_cast<HingeJoint*>(joints[jointsPerCar*i+1])->getPosition1Rate();
         len++;
      }
    }
	return len;
  }


  void CarChain::setMotorsIntern( const double* motors, int motornumber ) {
    int jointsPerCar = conf.supportWheels ? 4:2;
    int m=0;
    for( int i=0; i<conf.carNumber; i++){
        dynamic_cast<HingeJoint*>(joints[jointsPerCar*i])->addForce1( motors[m]*conf.wheelRadius );
        m++;
        dynamic_cast<HingeJoint*>(joints[jointsPerCar*i+1])->addForce1( motors[m]*conf.wheelRadius );
        m++;
    }
    if( conf.spring ) {
        for( int i=0; i<conf.carNumber-1; i++) {
            int jNumber = jointsPerCar*conf.carNumber + i;
            double phi = dynamic_cast<UniversalJoint*>(joints[jNumber])->getPosition1();
            double springForce = -conf.springConst *phi ;
            dynamic_cast<UniversalJoint*>(joints[jNumber])->addForce1(springForce);
        }
    }
  }


  void CarChain::create(const Matrix& pose) {
    /** Creating new space for the chain with inside collision of all elements */
    //odeHandle.createNewSimpleSpace(parentspace, false);
    odeHandle.createNewHashSpace(parentspace, false);
    spaces.resize( conf.carNumber );

    vector<Cylinder*> bodies;        /** bodies for the cars */
    bodies.resize( conf.carNumber );

    vector<Cylinder*> wheels;        /** wheels of the cars */
    wheels.resize( 2*conf.carNumber );
    vector<HingeJoint*> wheelJoints; /** joints between wheels and bodies */
    wheelJoints.resize( 2*conf.carNumber );

    vector<Sphere*> supWheels;        /** support wheels of the cars */
    supWheels.resize( 2*conf.carNumber );
    vector<BallJoint*> supWheelJoints; /** joints between support wheels and bodies */
    supWheelJoints.resize( 2*conf.carNumber );

    vector<Joint*> carJoints;        /** connections between cars */
    carJoints.resize( conf.carNumber-1 );
   
    /*******************
    N = parts per car    i=0 to i<conf.carNumber  CA= carNumber
    objects[N*i]         -  bodies
    objects[N*i+1]       -  left wheels
    objects[N*i+2]       -  right wheels
    evtl. objects[N*i+3] -  support wheels front
    evtl. objects[N*i+4] -  support wheels back
    joints[N*i]          -  left wheel joint
    joints[N*i+1]        -  right wheel joint
    evtl. joints[N*i+2]  -  support wheel joint front
    evtl. joints[N*i+3]  -  support wheel joint back
    joints[(2o4)*CA + i] -  joints between cars
    joints[(2o4)*CA + CA-1] 
    */

    for( int i=0; i<conf.carNumber; i++) {

        OdeHandle o(odeHandle);
        //o.createNewSimpleSpace( odeHandle.space,true );
        o.createNewHashSpace( odeHandle.space,true );
        spaces[i] = o;

        /** Creating body */
        bodies[i] = new Cylinder( conf.bodyRadius, conf.bodyHeight );
        //bodies[i]->setTexture("Images/chess.rgb");
        bodies[i]->init( spaces[i], conf.bodyMass, osgHandle.changeColor(Color(1,1.2,0)) );
        Matrix bodyPos = Matrix::translate(0,-i*conf.carDistance*conf.bodyRadius,0)*pose;
        bodies[i]->setPose( bodyPos );
        objects.push_back(bodies[i]);

        /** Creating Left Wheels */
        wheels[2*i] = new Cylinder( conf.wheelRadius, conf.wheelHeight );
        wheels[2*i]->setTexture("Images/chess.rgb");
        wheels[2*i]->init( spaces[i], conf.wheelMass, osgHandle );
        Matrix lwPos = Matrix::rotate(M_PI/2.,0.,1.,0.)*
                       Matrix::translate( conf.bodyRadius+conf.wheelHeight/2.,0.,0.)*
                       bodyPos;
        wheels[2*i]->setPose( lwPos );
        objects.push_back( wheels[2*i] );
        /** Creating Wheel Joints */
        wheelJoints[2*i] = new HingeJoint( bodies[i], wheels[2*i], wheels[2*i]->getPosition(), Axis(0,0,1)*lwPos );
        wheelJoints[2*i]->init( spaces[i], osgHandle );
        joints.push_back( wheelJoints[2*i] );

        /** Creating Right Wheels */
        wheels[2*i+1] = new Cylinder( conf.wheelRadius, conf.wheelHeight );
        wheels[2*i+1]->setTexture("Images/chess.rgb");
        wheels[2*i+1]->init( spaces[i], conf.wheelMass, osgHandle );
        Matrix rwPos = Matrix::rotate(M_PI/2.,0.,1.,0.)*
                       Matrix::translate( -(conf.bodyRadius+conf.wheelHeight/2.),0.,0.)*
                       bodyPos;
        wheels[2*i+1]->setPose( rwPos );
        objects.push_back( wheels[2*i+1] );
        /** Creating Wheel Joints */
        wheelJoints[2*i+1] = new HingeJoint( bodies[i], wheels[2*i+1], wheels[2*i+1]->getPosition(), Axis(0,0,1)*rwPos );
        wheelJoints[2*i+1]->init( spaces[i], osgHandle );
        joints.push_back( wheelJoints[2*i+1] );

        /** Creating Support Wheels */
        if( conf.supportWheels ) {        
                /** Creating Left Support Wheels */
                supWheels[2*i] = new Sphere( conf.supWheelRadius );
                supWheels[2*i]->init( spaces[i], conf.supWheelMass, osgHandle );
                Matrix lwPos = Matrix::translate( 0., conf.bodyRadius-2.*conf.supWheelRadius, conf.supWheelAnchor )*
                               bodyPos;
                supWheels[2*i]->setPose( lwPos );
                objects.push_back( supWheels[2*i] );
                /** Creating Joints */
                supWheelJoints[2*i] = new BallJoint( bodies[i], supWheels[2*i], supWheels[2*i]->getPosition() );
                supWheelJoints[2*i]->init( spaces[i], osgHandle, true, conf.supWheelRadius/2. );
                joints.push_back( supWheelJoints[2*i] );
                /** Creating Right Wheels */
                supWheels[2*i+1] = new Sphere( conf.supWheelRadius );
                supWheels[2*i+1]->init( spaces[i], conf.supWheelMass, osgHandle );
                Matrix rwPos = Matrix::translate( 0., -conf.bodyRadius+2.*conf.supWheelRadius, conf.supWheelAnchor )*
                               bodyPos;
                supWheels[2*i+1]->setPose( rwPos );
                objects.push_back( supWheels[2*i+1] );
                /** Creating Wheel Joints */
                supWheelJoints[2*i+1] = new BallJoint( bodies[i], supWheels[2*i+1], supWheels[2*i+1]->getPosition() );
                supWheelJoints[2*i+1]->init( spaces[i], osgHandle, true, conf.supWheelRadius/2. );
                joints.push_back( supWheelJoints[2*i+1] );
        }
    }
    enum JType  {BallJ, Hinge2J, UniversalJ};
    JType cj = UniversalJ;
    switch (cj){
      case BallJ:
        for( int i=0; i<conf.carNumber-1; i++) {
            /** Creating joints between cars */
            Vec3 jointPos = Vec3(0., -conf.carDistance/2.-i*conf.carDistance*conf.bodyRadius, 0.)*pose;
            carJoints[i] = new BallJoint( bodies[i], bodies[i+1], jointPos );
            carJoints[i]->init( odeHandle, osgHandle, true, conf.bodyHeight/2.);
            joints.push_back( carJoints[i] );
        }
        break;
      case Hinge2J:
        for( int i=0; i<conf.carNumber-1; i++) {
            /** Creating joints between cars */
            Vec3 jointPos = Vec3(0., -conf.carDistance/2.-i*conf.carDistance*conf.bodyRadius, 0.)*pose;
            Matrix jP = Matrix::translate( Vec3(0., -conf.carDistance/2.-i*conf.carDistance*conf.bodyRadius, 0.)) *pose;
            carJoints[i] = new Hinge2Joint( bodies[i], bodies[i+1], jointPos, Axis(0,0,1)*jP, Axis(1,0,0)*jP );
            carJoints[i]->init( odeHandle, osgHandle, true, conf.bodyHeight/2.);
            carJoints[i]->setParam(dParamLoStop, -0.5);
            carJoints[i]->setParam(dParamHiStop, 0.5);
            joints.push_back( carJoints[i] );
        }
        break;
      case UniversalJ:
        for( int i=0; i<conf.carNumber-1; i++) {
            /** Creating joints between cars */
            Vec3 jointPos = Vec3(0., -conf.carDistance/2.-i*conf.carDistance*conf.bodyRadius, 0.)*pose;
            Matrix jP = Matrix::translate( Vec3(0., -conf.carDistance/2.-i*conf.carDistance*conf.bodyRadius, 0.)) *pose;
            carJoints[i] = new UniversalJoint( bodies[i], bodies[i+1], jointPos, Axis(0,0,1)*jP, Axis(1,0,0)*jP );
            carJoints[i]->init( odeHandle, osgHandle, true, conf.bodyHeight/2.);
            carJoints[i]->setParam(dParamLoStop, -0.5);
            carJoints[i]->setParam(dParamHiStop, 0.5);
            joints.push_back( carJoints[i] );
        }
        break;
    }

  }
}
