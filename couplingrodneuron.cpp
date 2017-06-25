#include "couplingrodneuron.h"
#include <assert.h>

using namespace std;
using namespace matrix;


/** Constructor */
CouplingRod::CouplingRod(const std::string& name, const lpzrobots::OdeConfig& odeconfig)
  : AbstractController(name, "1.0"), odeconfig(odeconfig)  { }


/** initialize function of controller to set all parameters
 * and add plot options as well as possibility to change parameters
 * from the console */
void CouplingRod::init(int sensornumber, int motornumber, RandGen* randGen) 
{ 
  // Set the number of sensors and motors
  nSensors = sensornumber;
  nMotors  = motornumber;

  time = 0;
  stepSize = odeconfig.simStepSize*odeconfig.controlInterval;
   
  addParameterDef("a",    &a,       2, "mode 0: slope of sigmoidal function");
  addParameterDef("b",    &b,       0, "mode 0: threshold of sigmoidal function");
  addParameterDef("k",    &k,       2, "couping rod: spring constant");
  addParameterDef("mode", &mode,    0, " sigmoidal (0) or sinus (1) "); 
  addParameterDef("A",    &A,     0.8, "mode 1: amplitude of sinus target"); 
  addParameterDef("f",    &frequ,   3, "mode 1; rotational frequency in [1/s] of the driving force");
  
  N.resize( nMotors ); 
  cout << "########" << endl << "Controller internal variables: " << endl;
  cout << "nNeurons =  nMotors: " << nMotors << "   and nSensors: "<< nSensors << "   and stepsize: " << stepSize << endl;

  for( int i=0; i<nMotors; i++) {
     N[i].x=0.;
     N[i].y=0.;
     /** Parameter Gamma is the same for all neurons */
     if(i==0) addParameterDef("Gamma", &N[i].gamma, 60., "mode 0; decay constant of membrane potential");
     //addParameterDef("n"+itos(i)+":gamma", &N[i].gamma, 20., "mode 4; decay constant of membrane potential");
     addInspectableValue("n"+itos(i)+":x_act", &N[i].x_act, "actual position of left x between [-1,1]");
     addInspectableValue("n"+itos(i)+":x_tar", &N[i].x_tar, "target position of left x between [-1,1]");
     addInspectableValue("n"+itos(i)+":x",     &N[i].x,     "mode 0: membrane potential");
  }
}


void CouplingRod::step(const sensor* sensors, int sensornumber,
                           motor* motors, int motornumber) {
  /** calculation of dt and adaptation if something was changed online in odeconfig*/
  double old_stepSize = stepSize;
  stepSize = odeconfig.simStepSize*odeconfig.controlInterval;
  if( old_stepSize!=stepSize ) cout << "Controller stepsize = " << stepSize << endl;
  /** generating motor values using coupling rod mechanism */
  /** loop over all wheels calculate x_tar and set motor value */
  for( int i=0; i<nMotors; i++) 
  {    
    N[i].x_act  = cos(sensors[i]);
    if(mode==0) { /** sigmoidal */
      /** all gammas are the same value */
      N[i].gamma  = N[0].gamma;
      N[i].x     += N[i].gamma *( N[i].x_act - N[i].x ) *stepSize;
      N[i].y      = y(N[i].x);
      N[i].x_tar  = 2.*N[i].y -1.;
    }
    if(mode==1) { /** sinus */
      N[i].x_tar  =   A*sin(2*M_PI*frequ*time);
    }
    /** F_tan (multiply with R in robot) */
    motors[i]  = couplingRod( N[i].x_tar, sensors[i] );
  }
  time += stepSize;
}


double CouplingRod::couplingRod(double x_tar, double phi) 
{ /** defines transmission of power to the wheels */
  return k * sin(phi) * ( x_tar - cos(phi) );
}


double CouplingRod::y(double x)
{ /** sigmoidal funcion */
  return 1./( 1.+exp( a*(b-x) ) );
}


/************************** NOT USED ******************/

void CouplingRod::stepNoLearning(const sensor* sensors, int number_sensors,
                                     motor* motors, int number_motors) { }


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
