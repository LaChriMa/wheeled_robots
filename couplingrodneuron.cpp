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

#include "couplingrodneuron.h"

#include <assert.h>
using namespace std;
using namespace matrix;

CouplingRod::CouplingRod(const std::string& name, const lpzrobots::OdeConfig& odeconfig)
  : AbstractController(name, "1.0"), odeconfig(odeconfig)  { }


void CouplingRod::step(const sensor* sensors, int sensornumber,
                           motor* motors, int motornumber) {
  double old_stepSize = stepSize;
  stepSize = odeconfig.simStepSize*odeconfig.controlInterval;
  if( old_stepSize!=stepSize ) cout << " Controller internal stepsize = " << stepSize << endl;
  // generating motor values
  stepNoLearning(sensors,sensornumber, motors, motornumber);

}


void CouplingRod::stepNoLearning(const sensor* sensors, int number_sensors,
                                     motor* motors, int number_motors)
{
   for( int i=0; i<number_motors; i++) 
   {    
       /** all gammas are the same value */
       N[i].gamma=N[0].gamma;
       /** adapting gamma to stepSize diffrent form 0.001 */
       double new_gamma = N[i].gamma*1000*stepSize;

       N[i].x_act  =   cos(sensors[i]);
       N[i].x     +=   new_gamma *( N[i].x_act - N[i].x ) *stepSize;
       N[i].y      =   y(N[i].x);
       N[i].x_tar  =   2. *N[i].y -1.;
       motors[i]   =   couplingRod( N[i].x_tar, sensors[i] );
   }
  time += stepSize;
}


double CouplingRod::couplingRod(double x_tar, double phi) 
{ // defines transmission of power to the wheels
  return k * sin(phi) * ( x_tar - cos(phi) );
}


double CouplingRod::y(double x)
{ // sigmoidal funcion  
  return 1./( 1.+exp( a*(b-x) ) );
}



void CouplingRod::init(int sensornumber, int motornumber, RandGen* randGen) {
  // Set the number of sensors and motors
  nSensors = sensornumber;
  nMotors  = motornumber;

  stepSize = odeconfig.simStepSize*odeconfig.controlInterval;
  time = 0;

  addParameterDef("a", &a, 2, "mode 0 & 4; slope of sigmoidal function");
  addParameterDef("b", &b, 0, "mode 0 & 4; threshold of sigmoidal function");
  addParameterDef("k", &k, 2, "all modes; spring constant");
  //addParameterDef("mode", &mode, 4, "0: sigmoidal; 1: sinus; 2: speed-up; 4: membrane");
  //addParameterDef("frequency", &frequ, 0.5, "mode 1; rotational frequency in [1/s] of the driving force");
  //addParameterDef("phaseShift", &delPhi, 0, "mode 0; phase shift x=cos(phi-delPhi) in y( phi )");
  
  N.resize( nMotors ); 

  cout << "########" << endl << "Controller internal variables: " << endl;
  cout << "nNeurons =  nMotors: " << nMotors << "   and nSensors: "<< nSensors << "   and stepsize: " << stepSize << endl;

  for( int i=0; i<nMotors; i++) {
     //if(i==0) addParameterDef("n"+itos(i)+":gamma", &N[i].gamma, 20., "mode 4; decay constant of membrane potential");
     if(i==0) addParameterDef("Gamma", &N[i].gamma, 20., "mode 4; decay constant of membrane potential");
     N[i].x=0.;
     addInspectableValue("n"+itos(i)+":x_act", &N[i].x_act,  "actual position of left x between [-1,1]");
     addInspectableValue("n"+itos(i)+":x_tar", &N[i].x_tar,  "target position of left x between [-1,1]");
     addInspectableValue("n"+itos(i)+":x", &N[i].x,  "mode 4; membrane potential");
  }
}


int CouplingRod::getSensorNumber() const {
  return nSensors;
}


int CouplingRod::getMotorNumber() const {
  return nMotors;
}


bool CouplingRod::store(FILE* f) const {
  //  S.store(f); // if S is a matrix::Matrix
  Configurable::print(f,0);
  return true;
}


bool CouplingRod::restore(FILE* f) {
  //  S.restore(f); // if S is a matrix::Matrix
  Configurable::parse(f);
  return true;
}
