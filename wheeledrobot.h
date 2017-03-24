#ifndef __WHEELED_ROB
#define __WHEELED_ROB

#include <ode_robots/oderobot.h>
#include <ode_robots/primitive.h>
#include <ode_robots/joint.h>
#include <ode_robots/angularmotor.h>

#include <ode_robots/raysensorbank.h>
#include <ode_robots/irsensor.h>

#include <ode_robots/odeconfig.h>
#include <vector>

namespace lpzrobots{

/** structure to hold configuration of the robot */
typedef struct{
  int carNumber;              // Number of cars
  double carDistance;
  double bodyRadius;          // Radius of the cylinder defining the body
  double bodyHeight;          // Height of the cylinder defining the body
  double bodyMass;            // Mass of the body
  double wheelRadius;         // Radius of the cylinder defining the wheel
  double wheelHeight;         // Height of the cylinder defining the wheel
  double wheelMass;           // Mass of the wheel
  bool randomInitWP;
  bool supportWheels;         
  double supWheelMass;       
  double supWheelRadius;
  double supWheelAnchor;
  bool speedSensors;
  double spC1;
  double spC2;
  double spD1;
  double spD2;
} WheeledRobConf;


/** WheeledRob robot: 
 * Each car has two separated wheel on each side of the body
 * The cars are connected by angle joints
 * Inherit from OdeRobot */
class WheeledRob: public OdeRobot {
public:
  // Structure to hold the configuration of the robot
  WheeledRobConf conf;
  
  /** Contrustructor */
  WheeledRob(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
           const lpzrobots::OdeConfig& odeconfig,
           const WheeledRobConf &conf = getDefaultConf(),
           const std::string& name = "WheeledRobot");
  
  /** Default configuration of the robot */
  static WheeledRobConf getDefaultConf(){
    WheeledRobConf conf;
    conf.carNumber          = 5;
    conf.carDistance        = 2.2;   /* will later be multiplied by bodyRadius */
    conf.bodyRadius         = 0.08;
    conf.bodyHeight         = 0.06;
    conf.bodyMass           = 1.;
    conf.wheelRadius        = 0.04;
    conf.wheelHeight        = 0.01;
    conf.wheelMass          = 0.1;
    conf.randomInitWP       = true; //M_PI/4.0;
    conf.supportWheels      = (carNumber == 1) ? false:true;
    conf.supWheelMass       = 0.00001;
    conf.supWheelRadius     = conf.wheelRadius/4.;
    conf.supWheelAnchor     = -conf.wheelRadius+conf.supWheelRadius; /* y of the anchor */
    conf.spC1               = 1.;
    conf.spC2               = 1.;
    conf.spD1               = 0.02;
    conf.spD2               = 0.02;
    conf.speedSensors       = true;
    return conf;
  }
  
  /** Destructor */
  virtual ~WheeledRob();
  
  /** Place the robot in the desired pose
   * @param pose desired 4x4 pose matrix */
  virtual void placeIntern(const osg::Matrix& pose) override;
  
  /** Create the robot in the desired pose
   * @param pose desired 4x4 pose matrix */
  virtual void create(const osg::Matrix& pose);
  
  int getSensorNumberIntern(){ return sensorNo; }; 
  int getSensorsIntern( sensor* sensors, int sensornumber ); 
  
  virtual int getMotorNumberIntern(){ return motorNo; };
  virtual void setMotorsIntern( const double* motors, int motornumber );
  
  void velocityFriction(double friction, bool firstCar);

private:
  int sensorNo;
  int motorNo;
  
  std::vector<double> carAngleH;
  std::vector<double> carAngleV;
  std::vector<double> InitWPos;
  std::vector<Cylinder*> bodies; /** to apply fricion on all cars in a train */
  
  std::vector<OdeHandle> spaces;
  double stepsize;
  const lpzrobots::OdeConfig& odeconfig;

};

} 


#endif
