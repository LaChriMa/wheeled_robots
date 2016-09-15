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

#include "basiccontroller.h"

#include <assert.h>
using namespace std;
using namespace matrix;

BasicController::BasicController(const std::string& name, const lpzrobots::OdeConfig& odeconfig)
  : AbstractController(name, "1.0"), odeconfig(odeconfig)
{
  initialised=false;

  stepSize = odeconfig.simStepSize*odeconfig.controlInterval;
  cout << " STEPSIZE OF CONTROLLER = " << stepSize << endl;
  time = 0;
  addParameterDef("a", &a, 1, "slope of sigmoidal function");
  addParameterDef("b", &b, 0, "threshold of sigmoidal function");
  addParameterDef("k", &k, 1, "spring constant between neuron and coupling rod");
  addParameterDef("frequency", &omega, 1, "rotational frequency of the driving force");
  addInspectableValue("y_leftWheel",  &y_leftWheel,  "sigmoidal function depending on phi (sensor) and a and b");
  addInspectableValue("y_rightWheel", &y_rightWheel,  "sigmoidal function depending on phi (sensor) and a and b");
  addInspectableValue("x_actual", &x_actual,  "actual position between [-1,1]");
  addInspectableValue("x_target", &x_target,  "target position between [-1,1]");
}


double BasicController::couplingRod(double y, double phi) 
{
    //return k*(2.*y-1.-cos(phi))/sin(phi);
    return k*(2.*y-1.-cos(phi))/sin(M_PI/2.-phi);
}


double BasicController::y(double x)
{
    return 1./(1.+exp(a*(b-x)));
}

void BasicController::stepNoLearning(const sensor* sensors, int number_sensors,
                                     motor* motors, int number_motors)
{
  if( true )
  {
     if( time<5 )
	 {
       motors[0]= 1;
       motors[1]= 1;
     }
	 else{
	   //y_leftWheel = y( cos(sensors[0]) );
	   //y_rightWheel = y( cos(sensors[1]) );
	   y_leftWheel =  sin(2*M_PI*omega*time)/2.+0.5;
	   y_rightWheel = sin(2*M_PI*omega*time)/2.+0.5;
	   x_actual = cos(sensors[0]);
	   x_target = 2.*y_leftWheel-1.;
       motors[0]= couplingRod( y_leftWheel, sensors[0] );
       motors[1]= couplingRod( y_rightWheel, sensors[1] );
     }
  }
  if( false )
  {
    double amplitude = 2;
	if(time<5)
	{
	  motors[0] = 1;
	  motors[1] = 1;
	}
	else{
      motors[0] = amplitude*sin(time);
      motors[1] = amplitude*sin(time);
	}
  }
  if( false )
  {
    if(time<1)
    {
      motors[0] = 0;
      motors[1] = 0;
    }
    else if(time<8)
    {
      motors[0] = 2;
      motors[1] = 2;
    }
    else if(time<10)
    {
      motors[0] = 0;
      motors[1] = 0;
    }
    else if(time<12)
	{
      motors[0] =-0.5;
      motors[1] =-0.5;
    }
    else if(time<14)
	{
      motors[0] =0.5;
      motors[1] =0.5;
    }  
	else if(time>14)
	{
	  motors[0] = 0;
	  motors[1] = 0;
	}
  }
  /** linear acceleration */
  //motors[MIdx("left motor")] =  stepSize;
  //motors[MIdx("right motor")] = stepSize;
 
  time += stepSize;
}

void BasicController::step(const sensor* sensors, int sensornumber,
                           motor* motors, int motornumber) {
  double old_stepSize = stepSize;
  stepSize = odeconfig.simStepSize*odeconfig.controlInterval;
  if( old_stepSize != stepSize) cout << " STEPSIZE OF CONTROLLER = " << stepSize << endl;
  stepNoLearning(sensors,sensornumber, motors, motornumber);
}


void BasicController::init(int sensornumber, int motornumber, RandGen* randGen) {
  //assert(motornumber >=2 && sensornumber >=8);
  // Set the number of sensors and motors
  nSensors = sensornumber;
  nMotors  = motornumber;
  initialised=true;
}


int BasicController::getSensorNumber() const {
  return nSensors;
}

int BasicController::getMotorNumber() const {
  return nMotors;
}

bool BasicController::store(FILE* f) const {
  //  S.store(f); // if S is a matrix::Matrix
  Configurable::print(f,0);
  return true;
}

bool BasicController::restore(FILE* f) {
  //  S.restore(f); // if S is a matrix::Matrix
  Configurable::parse(f);
  return true;
}
