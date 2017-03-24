#ifndef __COUPLING_ROD_NEURON
#define __COUPLING_ROD_NEURON

#include <selforg/abstractcontroller.h>
#include <ode_robots/odeconfig.h>
#include <vector>


class CouplingRod : public AbstractController{

public:

  CouplingRod(const std::string& name, const lpzrobots::OdeConfig& odeconfig);
  
  /** initialisation of the controller with the given sensor/ motornumber
    Must be called before use. The random generator is optional.  */
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0)  override;
  
  /** @return Number of sensors the controller
    was initialised with or 0 if not initialised */
  virtual int getSensorNumber() const override;
  
  /** @return Number of motors the controller
    was initialised with or 0 if not initialised */
  virtual int getMotorNumber() const override;
  
  /** performs one step.
    Calculates motor commands from sensor inputs.
    @param sensors sensors inputs scaled to [-1,1]
    @param sensornumber length of the sensor array
    @param motors motors outputs. MUST have enough space for motor values!
    @param motornumber length of the provided motor array */
  virtual void step(const sensor* sensors, int sensornumber,
                        motor* motors, int motornumber) override;
  
  /** performs one step without learning.
    @see step */
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors) override;
  
  /** stores the object to the given file stream (binary). */
  virtual bool store(FILE* f) const override;
  
  /** loads the object from the given file stream (binary). */
  virtual bool restore(FILE* f) override;
  
  /** Calculates tangential force resulting from the spring force */
  double couplingRod(double x_tar, double phi);
  
  /** sigmoidal transfer function with x="membrane potential" */
  double y(double x);	


protected:
  double nSensors;
  double nMotors;

private:
  const lpzrobots::OdeConfig& odeconfig;
  double time;
  double stepSize;
  
  /** coupling rod */
  double k;
  int mode;
  
  /** sinus */
  double frequ;
  double A;
  
  /** sigmoidal */
  double a;
  double b;
 
  struct Neuron {
    double x;
    double gamma;
    double y;
    double x_act;
    double x_tar;
  };
  std::vector<Neuron> N;
};

#endif // Header guard
