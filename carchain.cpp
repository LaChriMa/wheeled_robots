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
    sensorNo = (conf.speedSensors ? 2 : 1) * motorNo + conf.carNumber-1; /** each wheel has an angle and an angular velocity sensor */

    cout << "########" << endl << "Variables of the CarChain: " << endl;
    cout << "motorNo: " << motorNo << "   and sensorNo: " << sensorNo << endl;
    /* storage for orientations of the cars, if all 0 they are on a straight line */
    carAngle.resize(conf.carNumber-1); 
    carAngle.assign(conf.carNumber-1, 0);
    //TODO adaptation of the internal stepsize to the global
    stepsize = 0.001;
    addParameter("stepsizeRobot", &this->stepsize, "Internal stepsize of the robot");
    addParameter("damping", &this->conf.springDamp, "Damping of the springs linking the cars");
    addParameter("springconstant", &this->conf.springConst, "Spring constant of the links between the cars");
  }


  CarChain::~CarChain() {}


  /** Calculates the position of the robot so that it doesn't stuck in the 
    * ground when it is created */
  void CarChain::placeIntern(const Matrix& pose) {
    assert(2.* conf.wheelRadius > conf.bodyHeight);
    Matrix initialPose = pose * Matrix::translate( Vec3(0.,0., conf.wheelRadius) );
    create(initialPose);
  }


  /** Gets the Joint values for wheel orientation and angular velocity 
    * of each wheel (support wheels are passive and therefore not included
    * The lenght of the sensor list is  (4* #cars) */
  int CarChain::getSensorsIntern( sensor* sensors, int sensornumber) {
    int len=0;
    int JPC = conf.supportWheels ? 4:2; /* Joints per car */
    for( int i=0; i<conf.carNumber; i++ ) 
    { /* Wheel orientation */
       sensors[len] = dynamic_cast<HingeJoint*>(joints[JPC*i+0])->getPosition1();
       len++;
       sensors[len] = dynamic_cast<HingeJoint*>(joints[JPC*i+1])->getPosition1();
       len++;
    }
    if( conf.speedSensors ) {
      for( int i=0; i<conf.carNumber; i++) 
      { /* Wheel velocities */
         sensors[len] = dynamic_cast<HingeJoint*>(joints[JPC*i+0])->getPosition1Rate();
         len++;
         sensors[len] = dynamic_cast<HingeJoint*>(joints[JPC*i+1])->getPosition1Rate();
         len++;
      }
    }
    for( int i=0; i<conf.carNumber-1; i++)
    {  /** Angles between cars */
       sensors[len] =carAngle[i];    
       len++;
    }    
	return len;
  }


  /** The motor values which were given by the controller are tangential forces. 
    * multiplied by the wheelRadius gives the torques. 
    * If a spring concatenates the cars the force corresponding to a damped spring
    * must be calculated in the second for-loop
    */
  void CarChain::setMotorsIntern( const double* motors, int motornumber ) {
    int JPC = conf.supportWheels ? 4:2;
    int m=0;
    for( int i=0; i<conf.carNumber; i++)
    { /* wheel motors */
      dynamic_cast<HingeJoint*>(joints[JPC*i])->addForce1( motors[m]*conf.wheelRadius );
      m++;
      dynamic_cast<HingeJoint*>(joints[JPC*i+1])->addForce1( motors[m]*conf.wheelRadius );
      m++;
    }
    int n=0;
    for( int j=JPC*conf.carNumber; j<conf.carNumber*(JPC+1)-1; j++) 
    { /* spring force */
      double angle_new = dynamic_cast<UniversalJoint*>(joints[j])->getPosition1();
      double springForce = -conf.springConst*angle_new  -conf.springDamp*(angle_new - carAngle[n])/stepsize;
      dynamic_cast<UniversalJoint*>(joints[j])->addForce1(springForce;
      carAngle[n] = angle_new;
      n++;
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
        wheelJoints[2*i]->init( spaces[i], osgHandle, false );
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
        wheelJoints[2*i+1]->init( spaces[i], osgHandle, false );
        joints.push_back( wheelJoints[2*i+1] );

        if( conf.supportWheels ) {        
           /** Creating Left Support Wheels */
           supWheels[2*i] = new Sphere( conf.supWheelRadius );
           supWheels[2*i]->init( spaces[i], conf.supWheelMass, osgHandle );
           Matrix fwPos = Matrix::translate( 0., conf.bodyRadius-2.*conf.supWheelRadius, conf.supWheelAnchor )*
                          bodyPos;
           supWheels[2*i]->setPose( fwPos );
           objects.push_back( supWheels[2*i] );
           /** Creating Joints */
           supWheelJoints[2*i] = new BallJoint( bodies[i], supWheels[2*i], supWheels[2*i]->getPosition() );
           supWheelJoints[2*i]->init( spaces[i], osgHandle, true, conf.supWheelRadius/2. );
           joints.push_back( supWheelJoints[2*i] );
           /** Creating Right Wheels */
           supWheels[2*i+1] = new Sphere( conf.supWheelRadius );
           supWheels[2*i+1]->init( spaces[i], conf.supWheelMass, osgHandle );
           Matrix bwPos = Matrix::translate( 0., -conf.bodyRadius+2.*conf.supWheelRadius, conf.supWheelAnchor )*
                          bodyPos;
           supWheels[2*i+1]->setPose( bwPos );
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
            //Vec3 jointPos = Vec3(0., -conf.carDistance/2.-i*conf.carDistance*conf.bodyRadius, 0.)*pose;
            //Matrix jP = Matrix::translate( Vec3(0., -conf.carDistance/2.-i*conf.carDistance*conf.bodyRadius, 0.)) *pose;
            Vec3 jointPos = Vec3(0., -(i+0.5)*conf.carDistance*conf.bodyRadius, conf.wheelRadius);
            Matrix jP = Matrix::translate( Vec3(0., -(i+0.5)*conf.carDistance*conf.bodyRadius, conf.wheelRadius));
            carJoints[i] = new UniversalJoint( bodies[i], bodies[i+1], jointPos, Axis(0,0,1)*jP, Axis(1,0,0)*jP );
            carJoints[i]->init( odeHandle, osgHandle, true, conf.bodyHeight/1.5);
            carJoints[i]->setParam(dParamLoStop, -0.8);
            carJoints[i]->setParam(dParamHiStop, 0.8);
            joints.push_back( carJoints[i] );
        }
        break;
    }

  }


  //void CarChain::notifyOnChange(const paramkey& key) {
  //  
  //}



}


