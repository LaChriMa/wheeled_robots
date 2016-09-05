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

#include "barrel.h"

#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/torquesensor.h>

// Using namespaces
using namespace osg;
using namespace std;
namespace lpzrobots{

  Barrel::Barrel(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                             const BarrelConf& conf, const string& name)
    : OdeRobot(odeHandle, osgHandle, name, "2.0"), conf(conf){
	addParameter("rollingFriction", &this->conf.rollingFriction, " ");
	addParameter("torque", &this->conf.torque, " ");
	addParameter("time", &this->time, "internal counter for time");
  }


  Barrel::~Barrel(){
  }


  void Barrel::placeIntern(const Matrix& pose){
    /** Moving robot upward such that the wheel are not stuck on the ground */
    Matrix initialPose;
	initialPose = pose * Matrix::translate(Vec3(0, 0, 3) );
    create(initialPose);
  }


  int Barrel::getSensorsIntern( sensor* sensors, int sensornumber) {
    dBodyID b = getMainPrimitive()->getBody();
    const double* vel = dBodyGetAngularVel( b);
	sensors[0]= -vel[0]*conf.rollingFriction;
	return 1;
  }


  void Barrel::setMotorsIntern( const double* motors, int motornumber ) {
    dBodyID b = getMainPrimitive()->getBody();
    const double* vel = dBodyGetAngularVel( b);
	if( time < 2 ) dBodyAddTorque ( b , 0.5, 0, 0 );
	else if( time > 4 and time < 8 ) {
	    dBodyAddTorque( b, -vel[0]*conf.rollingFriction, 
	   		 			   -vel[1]*conf.rollingFriction, 
	   					   -vel[2]*conf.rollingFriction );
	}
	else if( time > 8 and time < 12) {
		dBodyAddTorque ( b , conf.torque, 0, 0 );
	    dBodyAddTorque( b, -vel[0]*conf.rollingFriction, 
	   		 			   -vel[1]*conf.rollingFriction, 
	   					   -vel[2]*conf.rollingFriction );
	}
	else if( time > 12 and time < 16) {
		dBodyAddTorque ( b , -conf.torque, 0, 0 );
	}
	//else {
	//   /** apply torque M */
	//   if(conf.torque != 0) dBodyAddTorque ( b , conf.torque, 0, 0 );
	//   /** rolling friction proportional to the velocity */
	//   if(conf.rollingFriction != 0){
	//    	dBodyAddTorque( b, -vel[0]*conf.rollingFriction, 
	//   			 			   -vel[1]*conf.rollingFriction, 
	//   						   -vel[2]*conf.rollingFriction );
	//   }
	//}
	time +=0.01;
    //double friction = odeHandle.substance.roughness;
    //if(fabs(vel[2])>0.2){
    //  dBodyAddTorque ( b , 0 , 0 , -0.05*friction*vel[2] );
    //} 
  }


  void Barrel::create(const Matrix& pose) {
    /* Creating body */
    auto body = new Cylinder(conf.bodyRadius, conf.bodyHeight);
    body->init(odeHandle, conf.bodyMass, osgHandle.changeColor(Color(1,1,0)));
    body->setPose( Matrix::rotate(M_PI/2.0,0,1,0)*Matrix::translate(0,0,conf.bodyRadius) );
    objects.push_back(body);

  }
}
