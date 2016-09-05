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
#ifndef __BARREL
#define __BARREL

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
    double bodyRadius;          // Radius of the cylinder defining the body
    double bodyHeight;          // Height of the cylinder defining the body
    double bodyMass;            // Mass of the body
	double rollingFriction;
	double torque;
  } BarrelConf;

  /**
   * Barrel robot: two separated wheel on each side of the body
   * Inherit from OdeRobot
   */
  class Barrel: public OdeRobot {
    public:
     // Structure to hold the configuration of the robot
     BarrelConf conf;

     /**
      * Contrustructor
      */
     Barrel(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                  const BarrelConf &conf = getDefaultConf(),
                  const std::string& name = "Barrel");

     /**
      * Default configuration of the robot
      */
     static BarrelConf getDefaultConf(){
       BarrelConf conf;
       conf.bodyRadius         = 1.;
       conf.bodyHeight         = .5;
       conf.bodyMass           = 1.;
	   conf.rollingFriction	   = 0.3;
	   conf.torque			   = 1;
       return conf;
     }

     /**
      * Destructor
      */
     virtual ~Barrel();

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

	 int getSensorNumberIntern(){ return 1; }; 
	 int getSensorsIntern( sensor* sensors, int sensornumber ); 

	 virtual int getMotorNumberIntern(){ return 2; };
	 virtual void setMotorsIntern( const double* motors, int motornumber );

	double time;

  };


} // end namespace lpzrobots


// End of header guard
#endif
