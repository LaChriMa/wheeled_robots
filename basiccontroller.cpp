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
  stepSize = odeconfig.simStepSize*odeconfig.controlInterval;
  cout << " STEPSIZE OF CONTROLLER = " << stepSize << endl;
  time = 0;

  /** initialization of membrane potential */
  x_l=-2; 
  x_r=3;

  addParameterDef("a", &a, 2, "mode 0 & 4; slope of sigmoidal function");
  addParameterDef("b", &b, 0, "mode 0 & 4; threshold of sigmoidal function");
  addParameterDef("k", &k, 2, "all modes; spring constant");
  addParameterDef("gamma_l", &gamma_l, 2., "mode 4; decay constant of membrane potential");
  addParameterDef("gamma_r", &gamma_r, 2., "mode 4; decay constant of membrane potential");
  addParameterDef("mode", &mode, 4, "0: sigmoidal; 1: sinus; 2: speed-up; 4: membrane");
  addParameterDef("phaseShift", &delPhi, 0, "mode 0; phase shift x=cos(phi-delPhi) in y( phi )");
  addParameterDef("frequency", &frequ, 0.5, "mode 1; rotational frequency in [1/s] of the driving force");

  addInspectableValue("x_act_l", &x_act_l,  "actual position of left x between [-1,1]");
  addInspectableValue("x_tar_l", &x_tar_l,  "target position of left x between [-1,1]");
  addInspectableValue("x_act_r", &x_act_r,  "actual position of right x between [-1,1]");
  addInspectableValue("x_tar_r", &x_tar_r,  "target position of right x between [-1,1]");
  addInspectableValue("x_l", &x_l,  "mode 4; membrane potential");
  addInspectableValue("x_r", &x_r,  "mode 4; membrane potential");
}


double BasicController::couplingRod(double x_tar, double phi) 
{
  return k * sin(phi) * ( x_tar - cos(phi) );
}


double BasicController::y(double phi, double deltaPhi)
{ // sigmoidal funcion,  where x=cos(phi-dphi)
  double x = cos(phi-deltaPhi);
  return 1./( 1.+exp( a*(b-x) ) );
}


double BasicController::y(double x)
{ // sigmoidal funcion  
  return 1./( 1.+exp( a*(b-x) ) );
}


void BasicController::stepNoLearning(const sensor* sensors, int number_sensors,
                                     motor* motors, int number_motors)
{
  if( mode==0 ) /** transfer function with phase shift */
  {   
    y_l = y( sensors[0], delPhi );
    y_r = y( sensors[1], delPhi );

    x_act_l = cos(sensors[0]);
    x_tar_l = 2.*y_l-1.;
    x_act_r = cos(sensors[1]);
    x_tar_r = 2.*y_r-1.;

    motors[0] = couplingRod( x_tar_l, sensors[0] );
    motors[1] = couplingRod( x_tar_r, sensors[1] );
  }
  else if( mode==1 ) /** simple sinus, no sensorimotor loop */
  {
    x_act_l = cos(sensors[0]);
    x_tar_l = 0.8 *sin(2*M_PI*frequ*time);
    x_act_r = cos(sensors[1]);
    x_tar_r = 0.8 *sin(2*M_PI*frequ*time);

    motors[0] = couplingRod( x_tar_l, sensors[0] );
    motors[1] = couplingRod( x_tar_r, sensors[1] );
  }
  else if( mode==2 ) /** simple torque to the wheels */
  {
    motors[0] = 1.;
    motors[1] = 0;
  }
  else if( mode==4 ) /** sensorimotor loop with membrane potential */
  {  
    x_act_l = cos(sensors[0]);
    x_act_r = cos(sensors[1]);

    x_l += gamma_l*( x_act_l - x_l )*stepSize;
    x_r += gamma_r*( x_act_r - x_r )*stepSize;

    y_l = y( x_l );
    y_r = y( x_r );

    x_tar_l = 2.*y_l-1.;
    x_tar_r = 2.*y_r-1.;

    motors[0] = couplingRod( x_tar_l, sensors[0] );
    motors[1] = couplingRod( x_tar_r, sensors[1] );
  }
  
  time += stepSize;
}


void BasicController::step(const sensor* sensors, int sensornumber,
                           motor* motors, int motornumber) {
  // counter for time
  double old_stepSize = stepSize;
  stepSize = odeconfig.simStepSize*odeconfig.controlInterval;
  if( old_stepSize != stepSize) cout << " STEPSIZE OF CONTROLLER = " << stepSize << endl;
  // generating motor values
  stepNoLearning(sensors,sensornumber, motors, motornumber);
}


void BasicController::init(int sensornumber, int motornumber, RandGen* randGen) {
  // Set the number of sensors and motors
  nSensors = sensornumber;
  nMotors  = motornumber;
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
