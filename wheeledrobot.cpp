#include "wheeledrobot.h"
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/torquesensor.h>
#include <random>

using namespace osg;
using namespace std;
namespace lpzrobots{


WheeledRob::WheeledRob(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                   const lpzrobots::OdeConfig& odeconfig,
                   const WheeledRobConf& conf, const string& name)
  : OdeRobot(odeHandle, osgHandle, name, "2.0"), odeconfig(odeconfig), conf(conf) {
 
  motorNo = 2*conf.carNumber;  /** each car has two wheels */
  /** Number of sensors: 
   * Angle for each wheel                 ~ motorNo
   * opt. Angular velocity                ~ motorNo           if speedSensors
   * Angles between the cars horizontal   ~ carNumber-1
   * opt. Angles vertical                 ~ carNumber-1       if no supportWheels
   */
  sensorNo = (conf.speedSensors ? 2:1)*motorNo + (conf.supportWheels ? 1:2)*(conf.carNumber-1);
  cout << "########" << endl << "# Motor and Sensor number of Wheeled Robot: " << endl;
  cout << "# motorNo: " << motorNo << "   and sensorNo: " << sensorNo << endl;

  /** storage for orientations of the cars, if all 0 they are on a straight line */
  carAngleV.assign(conf.carNumber-1, 0);
  carAngleH.assign(conf.carNumber-1, 0);
  
  /** Initial wheel orientations */
  InitWPos.assign(motorNo, 0.);
  if(conf.randomInitWP) {
    random_device rd;
    mt19937 mt(rd());
    uniform_real_distribution<double> distribution(0,2*M_PI); /** half open interval [0; 2*M_PI) */
    for( int i=0; i<motorNo; i++) 
    {
      InitWPos[i]= distribution(mt);
      if(InitWPos[i]<M_PI) cout << "  " << InitWPos[i]/M_PI << " pi" ;
      else cout << "  " << (InitWPos[i]-2*M_PI)/M_PI << " pi" ;
    }
    cout << endl;
  }

  /** stepsize */
  stepsize = odeconfig.simStepSize*odeconfig.controlInterval;
  //TODO: damping constant of car joints the same for both angles
  //addParameter("verticalDamping", &this->conf.spD1, "Damping of carjoints around Z-AXIS");
  //addParameter("verticalSpring", &this->conf.spC1, "Spring constant");
  //if( !conf.supportWheels ) {
  //  addParameter("horizontalDamping", &this->conf.spD2, "Damping of carjoints on xy-plane");
  //  addParameter("horizontalSpring", &this->conf.spC2, "Spring constant");
  //}
  addParameter("damping", &this->conf.spD1, "Damping of car joints");
  addParameter("spring", &this->conf.spC1, "Spring constant of car joints");


}


WheeledRob::~WheeledRob() {}


/** Calculates the position of the robot so that it doesn't stuck in the 
  * ground when it is created */
void WheeledRob::placeIntern(const Matrix& pose) {
  assert(2.*conf.wheelRadius > conf.bodyHeight);
  Matrix initialPose = pose * Matrix::translate( Vec3(0.,0., conf.wheelRadius) );
  create(initialPose);
}


/** Gets the Joint values for wheel orientation and angular velocity 
  * of each wheel (support wheels are passive and therefore not included
  * The lenght of the sensor list is  (4* #cars) */
int WheeledRob::getSensorsIntern( sensor* sensors, int sensornumber) {
  int len=0;
  int JPC = conf.supportWheels ? 4:2; /* Joints per car */
  for( int i=0; i<conf.carNumber; i++ ) 
  { /* Wheel orientation */
     sensors[len] = dynamic_cast<HingeJoint*>(joints[JPC*i+0])->getPosition1() + InitWPos[len];
     sensors[len] -= (sensors[len]<M_PI) ? 0 : 2.*M_PI;
     len++;
     sensors[len] = dynamic_cast<HingeJoint*>(joints[JPC*i+1])->getPosition1() + InitWPos[len];
     sensors[len] -= (sensors[len]<M_PI) ? 0 : 2.*M_PI;
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
  {  /** Vertical angles between cars */
     sensors[len] = carAngleV[i];    
     len++;
  }
  if( !conf.supportWheels ) {
    for( int i=0; i<conf.carNumber-1; i++)
    {  /** Horizontal angles between cars */
       sensors[len] = carAngleH[i];    
       len++;
    }    
  }
  return len;
}


/** The motor values which were given by the controller are tangential forces. 
  * multiplied by the wheelRadius gives the torques. 
  * If a spring concatenates the cars the force corresponding to a damped spring
  * must be calculated in the second for-loop
  */
void WheeledRob::setMotorsIntern( const double* motors, int motornumber ) {
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
  stepsize = odeconfig.simStepSize*odeconfig.controlInterval;
  /** if train: loop over all connections between cars */
  for( int j=JPC*conf.carNumber; j<conf.carNumber*(JPC+1)-1; j++) 
  { /* spring force for vertical car angles */
    double angle_new = dynamic_cast<UniversalJoint*>(joints[j])->getPosition1();
    double springForce = -conf.spC1*angle_new - conf.spD1*(angle_new-carAngleV[n])/stepsize;
    dynamic_cast<UniversalJoint*>(joints[j])->addForce1(springForce);
    carAngleV[n] = angle_new;
    n++;
  }
  if( !conf.supportWheels ) {
    int n=0;
    for( int j=JPC*conf.carNumber; j<conf.carNumber*(JPC+1)-1; j++) 
    { /* spring force for horizontal car angles*/
      double angle_new = dynamic_cast<UniversalJoint*>(joints[j])->getPosition2();
      //TODO: damping constant of car joints the same for both angles
      double springForce = -conf.spC1*angle_new - conf.spD1*(angle_new-carAngleH[n])/stepsize;
      //double springForce = -conf.spC2*angle_new - conf.spD2*(angle_new-carAngleH[n])/stepsize;
      dynamic_cast<UniversalJoint*>(joints[j])->addForce2(springForce);
      carAngleH[n] = angle_new;
      n++;
    }
  }
}


/** Call to apply a friction to ALL or only FIRST body of the train */ 
void WheeledRob::velocityFriction(double friction, bool firstCar) {
  vector<Cylinder*>::iterator it;
  if( firstCar ) {
    it=bodies.begin();
    dBodyID b = (*it)->getBody();
    const double* vel = dBodyGetLinearVel(b);
    dBodyAddForce( b, -vel[0]*friction,
                       -vel[1]*friction,
                       -vel[2]*friction );
  }
  else {
    for( it=bodies.begin(); it!=bodies.end(); ++it ) {
      dBodyID b = (*it)->getBody();
      const double* vel = dBodyGetLinearVel(b);
      dBodyAddForce( b, -vel[0]*friction,
                         -vel[1]*friction,
                         -vel[2]*friction );
    }
  }
}


void WheeledRob::create(const Matrix& pose) {
  /** Creating new space for the chain with inside collision of all elements */
  odeHandle.createNewSimpleSpace(parentspace, false);
  //odeHandle.createNewHashSpace(parentspace, false);
  spaces.resize( conf.carNumber );

  //vector<Cylinder*> bodies;        /** bodies for the cars */
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
      o.createNewSimpleSpace( odeHandle.space,true );
      spaces[i] = o;
      OdeHandle* Space = &spaces[i];
      //OdeHandle* Space = &odeHandle;
      
      /** Creating body */
      bodies[i] = new Cylinder( conf.bodyRadius, conf.bodyHeight );
      bodies[i]->init( *Space, conf.bodyMass, i!=0 ? osgHandle.changeColor(Color(1,1.2,0)) : 
                                                        osgHandle.changeColor(Color(1.6,0.8,0)) );
      Matrix bodyPos = Matrix::translate(0,-i*conf.carDistance*conf.bodyRadius,0)*pose;
      bodies[i]->setPose( bodyPos );
      objects.push_back(bodies[i]);

      /** Creating Left Wheels */
      wheels[2*i] = new Cylinder( conf.wheelRadius, conf.wheelHeight );
      wheels[2*i]->setTexture("Images/chess.rgb");
      wheels[2*i]->init( *Space, conf.wheelMass, osgHandle );
      Matrix lwPos = Matrix::rotate(M_PI/2.,0.,1.,0.)*
                     Matrix::translate( conf.bodyRadius+conf.wheelHeight/2.,0.,0.)*
                     bodyPos;
      wheels[2*i]->setPose( lwPos );
      objects.push_back( wheels[2*i] );
      /** Creating Wheel Joints */
      wheelJoints[2*i] = new HingeJoint( bodies[i], wheels[2*i], wheels[2*i]->getPosition(), Axis(0,0,1)*lwPos );
      wheelJoints[2*i]->init( *Space, osgHandle, false );
      joints.push_back( wheelJoints[2*i] );

      /** Creating Right Wheels */
      wheels[2*i+1] = new Cylinder( conf.wheelRadius, conf.wheelHeight );
      wheels[2*i+1]->setTexture("Images/chess.rgb");
      wheels[2*i+1]->init( *Space, conf.wheelMass, osgHandle );
      Matrix rwPos = Matrix::rotate(M_PI/2.,0.,1.,0.)*
                     Matrix::translate( -(conf.bodyRadius+conf.wheelHeight/2.),0.,0.)*
                     bodyPos;
      wheels[2*i+1]->setPose( rwPos );
      objects.push_back( wheels[2*i+1] );
      /** Creating Wheel Joints */
      wheelJoints[2*i+1] = new HingeJoint( bodies[i], wheels[2*i+1], wheels[2*i+1]->getPosition(), Axis(0,0,1)*rwPos );
      wheelJoints[2*i+1]->init( *Space, osgHandle, false );
      joints.push_back( wheelJoints[2*i+1] );

      if( conf.supportWheels ) {        
         /** Creating Left Support Wheels */
         supWheels[2*i] = new Sphere( conf.supWheelRadius );
         supWheels[2*i]->init( *Space, conf.supWheelMass, osgHandle );
         Matrix fwPos = Matrix::translate( 0., conf.bodyRadius-2.*conf.supWheelRadius, conf.supWheelAnchor )*
                        bodyPos;
         supWheels[2*i]->setPose( fwPos );
         objects.push_back( supWheels[2*i] );
         /** Creating Joints */
         supWheelJoints[2*i] = new BallJoint( bodies[i], supWheels[2*i], supWheels[2*i]->getPosition() );
         supWheelJoints[2*i]->init( *Space, osgHandle, true, conf.supWheelRadius/2. );
         joints.push_back( supWheelJoints[2*i] );
         /** Creating Right Wheels */
         supWheels[2*i+1] = new Sphere( conf.supWheelRadius );
         supWheels[2*i+1]->init( *Space, conf.supWheelMass, osgHandle );
         Matrix bwPos = Matrix::translate( 0., -conf.bodyRadius+2.*conf.supWheelRadius, conf.supWheelAnchor )*
                        bodyPos;
         supWheels[2*i+1]->setPose( bwPos );
         objects.push_back( supWheels[2*i+1] );
         /** Creating Wheel Joints */
         supWheelJoints[2*i+1] = new BallJoint( bodies[i], supWheels[2*i+1], supWheels[2*i+1]->getPosition() );
         supWheelJoints[2*i+1]->init( *Space, osgHandle, true, conf.supWheelRadius/2. );
         joints.push_back( supWheelJoints[2*i+1] );
      }
  }
  /** The Universal Joints are so far best choice to connect the cars.
    * One can add seperate torques on to axes. 
    * By confining the maximal angles we prevent from internal collisions
    */
  enum JType  {BallJ, Hinge2J, UniversalJ};
  JType cj = UniversalJ;
  switch (cj){
    case BallJ:
      //for( int i=0; i<conf.carNumber-1; i++) {
      //    /** Creating joints between cars */
      //    Vec3 jointPos = Vec3(0., -conf.carDistance/2.-i*conf.carDistance*conf.bodyRadius, 0.)*pose;
      //    carJoints[i] = new BallJoint( bodies[i], bodies[i+1], jointPos );
      //    carJoints[i]->init( odeHandle, osgHandle, true, conf.bodyHeight/2.);
      //    joints.push_back( carJoints[i] );
      //}
      break;
    case Hinge2J:
      //for( int i=0; i<conf.carNumber-1; i++) {
      //    /** Creating joints between cars */
      //    Vec3 jointPos = Vec3(0., -conf.carDistance/2.-i*conf.carDistance*conf.bodyRadius, 0.)*pose;
      //    Matrix jP = Matrix::translate( Vec3(0., -conf.carDistance/2.-i*conf.carDistance*conf.bodyRadius, 0.)) *pose;
      //    carJoints[i] = new Hinge2Joint( bodies[i], bodies[i+1], jointPos, Axis(0,0,1)*jP, Axis(1,0,0)*jP );
      //    carJoints[i]->init( odeHandle, osgHandle, true, conf.bodyHeight/2.);
      //    carJoints[i]->setParam(dParamLoStop, -0.5);
      //    carJoints[i]->setParam(dParamHiStop, 0.5);
      //    joints.push_back( carJoints[i] );
      //}
      break;
    case UniversalJ:
      for( int i=0; i<conf.carNumber-1; i++) {
          /** Creating joints between cars */
          Vec3 jointPos = Vec3(0., -(i+0.5)*conf.carDistance*conf.bodyRadius, conf.wheelRadius);
          Matrix jP = Matrix::translate( Vec3(0., -(i+0.5)*conf.carDistance*conf.bodyRadius, conf.wheelRadius));
          carJoints[i] = new UniversalJoint( bodies[i], bodies[i+1], jointPos, Axis(0,0,1)*jP, Axis(1,0,0)*jP );
          carJoints[i]->init( odeHandle, osgHandle, true, conf.bodyHeight/1.5);
          carJoints[i]->setParam(dParamLoStop, -M_PI/4. );
          carJoints[i]->setParam(dParamHiStop, M_PI/4. );
          joints.push_back( carJoints[i] );
      }
      break;
  }
}



}


