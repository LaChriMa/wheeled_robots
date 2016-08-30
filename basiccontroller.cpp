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
  : AbstractController(name, "1.0"), odeconfig(odeconfig) {
  initialised=false;
  // add threshold parameter to configurable parameters, setable on console
  //addParameterDef("threshold", &threshold, 0.2, 0, 1, "threshold for IR-sensor");
  stepSize = odeconfig.simStepSize*odeconfig.controlInterval;
  cout << " STEPSIZE OF CONTROLLER = " << stepSize << endl;
  time = 0;
  addParameterDef("a", &a, 1, "slope of sigmoidal function");
  addParameterDef("b", &b, 0, "threshold of sigmoidal function");
  addParameterDef("k", &k, 1, "spring constant between neuron and coupling rod");
}


double BasicController::couplingRod(double y, double phi) {
    return k*(2.*y-1.-cos(phi))*cos(phi-M_PI/2.);
  }


double BasicController::y(double x) {
    return 1./(1.+exp(a*(b-x)));
  }

void BasicController::stepNoLearning(const sensor* sensors, int number_sensors,
                                     motor* motors, int number_motors) {
  //if( time<1){
  if( time<2){
    motors[0]= 0.5;
    motors[1]= 0.5;
  }else{
    motors[0]= couplingRod( y( cos(M_PI-sensors[0]) ), sensors[0] );
    motors[1]= couplingRod( y( cos(M_PI-sensors[1]) ), sensors[1] );
  }
  /** sinus*sinus */
  //double amplitude = 100;
  //motors[MIdx("left motor")] =  amplitude*sin(stepSize)*sin(stepSize);
  //motors[MIdx("right motor")] = amplitude*sin(stepSize)*sin(stepSize);
  /** von 0 auf 50 */
  //if(time<10 or time>15)
  //{
  //  motors[0] =  0;
  //	motors[1] = 0;
  //}
  //else{
  //  motors[0] = 1;
  //  motors[1] = 1;
  //}
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
