/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
#include <assert.h>
#include <ode-dbl/ode.h>

#include <ode_robots/primitive.h>
#include <ode_robots/osgprimitive.h>

#include <ode_robots/joint.h>

#include "nimm4angle.h"


using namespace osg;

namespace lpzrobots {

  // constructor:
  // - give handle for ODE and OSG stuff
  // - size of robot, maximal used force and speed factor are adjustable
  // - sphereWheels switches between spheres or wheels as wheels
  //   (wheels are only drawn, collision handling is always with spheres)
  Nimm4Angle::Nimm4Angle(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
               const std::string& name,
               double size/*=1.0*/, double force /*=3*/, double speed/*=15*/,
               bool sphereWheels /*=true*/)
    : // calling OdeRobots construtor with name of the actual robot
      OdeRobot(odeHandle, osgHandle, name, "$Id$")
  {
    // robot is not created till now
    created=false;

    this->osgHandle.color = Color(2, 156/255.0, 0, 1.0f);

    max_force   = force*size*size;  /*NOT_USED*/
    this->speed = speed;

    this->sphereWheels = sphereWheels;

    height=size;
    length=size/2.5; // length of body
    width=size/2;  // diameter of body
    radius=size/6; // wheel radius
    wheelthickness=size/20; // thickness of the wheels (if wheels used, no spheres)
    cmass=8*size;  // mass of the body
    wmass=size;    // mass of the wheels
    sensorno=4;    // number of sensors
    motorno=4;     // number of motors
    segmentsno=5;  // number of segments of the robot

    wheelsubstance.toRubber(50);

  };


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void Nimm4Angle::setMotorsIntern(const double* motors, int motornumber){
    assert(created); // robot must exist
    //// for each motor the motorcommand (between -1 and 1) multiplied with speed
    //// is set and the maximal force to realize this command are set
    //for (int i=0; i<len; i++){
    //  joints[i]->setParam(dParamVel2, motors[i]*speed);
    //  joints[i]->setParam(dParamFMax2, max_force);
    //}
    for(int i=0; i<motornumber; i++)
    {
        dynamic_cast<Hinge2Joint*>(joints[i])->addForce1( motors[i]*radius );
    }
  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Nimm4Angle::getSensorsIntern(sensor* sensors, int sensornumber){
    assert(created); // robot must exist
    int len=4;
    for(int i=0; i<4; i++)
    {
        sensors[i] = dynamic_cast<Hinge2Joint*>(joints[i])->getPosition1();
    }
    return len;
  };


  void Nimm4Angle::placeIntern(const osg::Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // width*0.6 is added (without this the wheels and half of the robot will be in the ground)
    Matrix p2;
    p2 = pose * Matrix::translate(Vec3(0, 0, width*0.6));
    create(p2);
  };


  /**
   * updates the osg notes
   */
  void Nimm4Angle::update() {
    OdeRobot::update();
    assert(created); // robot must exist

    for (int i=0; i<segmentsno; i++) { // update objects
      objects[i]->update();
    }
    for (int i=0; i < 4; i++) { // update joints
      joints[i]->update();
    }

  };


  /** creates vehicle at desired pose
      @param pose matrix with desired position and orientation
  */
  void Nimm4Angle::create( const osg::Matrix& pose ){
    if (created) {  // if robot exists destroy it
      destroy();
    }
    // create car space
    odeHandle.createNewSimpleSpace(parentspace, true);
    objects.resize(5);  // 1 capsule, 4 wheels
    joints.resize(4); // joints between cylinder and each wheel

    OdeHandle wheelHandle(odeHandle);
    // make the material of the wheels a hard rubber
    wheelHandle.substance = wheelsubstance;
    // create cylinder for main body
    // initialize it with ode-, osghandle and mass
    // rotate and place body (here by -90° around the y-axis)
    // use texture 'wood' for capsule
    // put it into objects[0]
    Capsule* cap = new Capsule(width/2, length);
    cap->setTexture("Images/wood.rgb");
    cap->init(odeHandle, cmass, osgHandle);
    cap->setPose(Matrix::rotate(-M_PI/2, 0, 1, 0) * pose);
    objects[0]=cap;

    // create wheels
    /*   front
         -----
      1 |     | 2
        |     |
        |     |
      3 |     | 4
         -----
     */
    for (int i=1; i<5; i++) {
      // create sphere with radius
      // and initialize it with odehandle, osghandle and mass
      // calculate position of wheels(must be at desired positions relative to the body)
      // rotate and place body (here by 90Deg around the x-axis)
      // set texture for wheels
      Sphere* sph = new Sphere(radius);
      sph->setTexture("Images/wood.rgb");
      sph->init(wheelHandle, wmass, osgHandle.changeColor(Color(0.8,0.8,0.8)));
      Vec3 wpos = Vec3( ((i-1)/2==0?-1:1)*length/2.0,
                        ((i-1)%2==0?-1:1)*(width*0.5+wheelthickness),
                        -width*0.6+radius );
      sph->setPose(Matrix::rotate(M_PI/2, 0, 0, 1) * Matrix::translate(wpos) * pose);
      objects[i]=sph;
    }

    // generate 4 joints to connect the wheels to the body
    for (int i=0; i<4; i++) {
      Pos anchor(dBodyGetPosition (objects[i+1]->getBody()));
      joints[i] = new Hinge2Joint(objects[0], objects[i+1], anchor, Axis(0,1,0)*pose, Axis(0,0,1)*pose);
      //joints[i] = new HingeJoint(objects[0], objects[i+1], anchor, Axis(0,1,0)*pose);
      joints[i]->init(odeHandle, osgHandle, true, 2.01 * radius);
    }
    //for (int i=0; i<4; i++) {
    //  // set stops to make sure wheels always stay in alignment
    //  joints[i]->setParam(dParamLoStop, 0);
    //  joints[i]->setParam(dParamHiStop, 0);
    //}

    created=true; // robot is created
  };


  /** destroys vehicle and space
   */
  void Nimm4Angle::destroy(){
    if (created){
      cleanup();
      odeHandle.deleteSpace(); // destroy space
    }
    created=false; // robot does not exist (anymore)
  }

}
