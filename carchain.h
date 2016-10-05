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

// Header guard
#ifndef __CAR_CHAIN
#define __CAR_CHAIN

#include <ode_robots/oderobot.h>
#include <ode_robots/primitive.h>
#include <ode_robots/joint.h>
#include <ode_robots/angularmotor.h>

#include <ode_robots/raysensorbank.h>
#include <ode_robots/irsensor.h>

// Using name space lpzrobots
namespace lpzrobots{

  // structure to hold configuration of the robot
  typedef struct{
    int carNumber;              // Number of cars
    double carDistance;
    double bodyRadius;          // Radius of the cylinder defining the body
    double bodyHeight;          // Height of the cylinder defining the body
    double bodyMass;            // Mass of the body
    double wheelRadius;         // Radius of the cylinder defining the wheel
    double wheelHeight;         // Height of the cylinder defining the wheel
    double wheelMass;           // Mass of the wheel
	double initWheelOrientation;
	bool supportWheels;         
    double sWheelMass;       
	bool sphericalSupportWheels;
  } CarChainConf;

  /**
   * CarChain robot: 
   * Each car has two separated wheel on each side of the body
   * The cars are connected by angle joints
   * Inherit from OdeRobot
   */
  class CarChain: public OdeRobot {
    public:
     // Structure to hold the configuration of the robot
     CarChainConf conf;

     /**
      * Contrustructor
      */
     CarChain(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                  const CarChainConf &conf = getDefaultConf(),
                  const std::string& name = "CarChain");

     /**
      * Default configuration of the robot
      */
     static CarChainConf getDefaultConf(){
       CarChainConf conf;
       conf.carNumber          = 2;
       conf.carDistance        = 2.5;
       conf.bodyRadius         = 1.;
       conf.bodyHeight         = .5;
       conf.bodyMass           = 1.;
       conf.wheelRadius        = .3;
       conf.wheelHeight        = .1;
       conf.wheelMass          = 5.;
	   conf.initWheelOrientation = 0;//M_PI/4.0;
	   conf.supportWheels 		= false;
       conf.sWheelMass          = 0.00001;
	   conf.sphericalSupportWheels 		= false;
       return conf;
     }

     /**
      * Destructor
      */
     virtual ~CarChain();

     /**
      * Place the robot in the desired pose
      * @param pose desired 4x4 pose matrix
      */
     virtual void placeIntern(const osg::Matrix& pose) override;

     /**
      * Create the robot in the desired pose
      * @param pose desired 4x4 pose matrix
      */
     virtual void create(const osg::Matrix& pose);

	 int getSensorNumberIntern(){ return sensorNo; }; 
	 int getSensorsIntern( sensor* sensors, int sensornumber ); 

	 virtual int getMotorNumberIntern(){ return motorNo; };
	 virtual void setMotorsIntern( const double* motors, int motornumber );

	private:
     int sensorNo;
     int motorNo;
	 double motorVel;

  };


} // end namespace lpzrobots


// End of header guard
#endif
