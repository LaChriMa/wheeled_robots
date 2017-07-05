#include <ode_robots/simulation.h>
#include <selforg/noisegenerator.h>
#include <ode_robots/odeagent.h>
#include <ode_robots/speedsensor.h>

//#include "basiccontroller.h"
//#include "differential.h"
#include <selforg/one2onewiring.h>

// Environment:
#include <ode_robots/playground.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/randomobstacles.h>
#include <ode_robots/terrainground.h>

#include <selforg/agent.h>

#include "wheeledrobot.h"
#include "couplingrodneuron.h"

#include <string>

using namespace lpzrobots;

class ThisSim : public Simulation
{

public:
  double friction;  /** velocity depending friction factor */
  bool friction_first_body;  /** velocity depending friction factor */
  std::string env = "playground";  /** "slope", "wall", "playground", "halfpipe" or "no" */
  bool randObstacles = true;
  Pos initPos;
  int carNum = 5;
  
  ThisSim() {
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    addColorAliasFile("colors.txt");
    setGroundTexture("Images/whiteground.jpg");
  }

  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    /******** GLOBAL SETTINGS **********/
    global.odeConfig.setParam("simstepsize", 0.001);
    global.odeConfig.setParam("controlinterval", 1);
    global.odeConfig.setParam("gravity", -9.8);
    global.odeConfig.setParam("noise", 0);
    global.odeConfig.addParameterDef("friction", &friction, 0.1, "parameter for velocity depending friction");
    global.odeConfig.addParameterDef("friction_first_body", &friction_first_body, false, " 0 or 1, if 0 friction is applied to all robots of the chain");
    /******* END GLOBAL SETTINGS *********/
  
    /************** ENVIRONMENT **********/
    if( env == "wall" )
    { 
      OdeHandle wallHandle = odeHandle;
      wallHandle.substance.toPlastic(0.8);
      //wallHandle.substance.toRubber(25);
      //wallHandle.substance.toFoam(1);
      auto* wall1 = new Box(15,0.3,1.5);
      wall1->init( wallHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      wall1->setSubstance(  Substance::getPlastic(0.8) );
      wall1->setPosition( Pos(0,30,0) );
      auto* wall2 = new Box(15,0.3,1.5);
      wall2->init( wallHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      wall2->setSubstance(  Substance::getPlastic(0.8) );
      wall2->setPosition( Pos(0,-10,0) );
      initPos = Pos(0,0,0);
      setCameraHomePos(Pos(0.00853173, -2.64447, 0.673756),  Pos(-0.251526, -8.93542, 0));
      setCameraMode( Follow );
    }
    if( env == "boxes" )
    {  
      /** läuft noch nicht */
      //auto* box = new Box(0.2,0.2,0.2);
      //box->init( odeHandle, 2, osgHandle );
      //box->setSubstance(  Substance::getPlastic(0.8) );   //  roughness
      ////box->setSubstance(  Substance::getRubber(25) ); // hardness 
      ////box->setSubstance(  Substance::getFoam(1) );  // hardness (elasticity 0)
      //box->setPosition( Pos(0,10,0) );
      //setCameraHomePos(Pos(0.0352671, -2.84712, 0.905692),  Pos(1.41342, -10.5767, 0));
    }
    if( env == "halfpipe" )
    {
      TerrainGround* pipe = new TerrainGround( odeHandle, osgHandle, "terrains/dip128.ppm",
                                               "terrains/dip128_texture.ppm", 10, 1000, 1,
                                               OSGHeightField::Red);
      pipe->setPose( osg::Matrix::translate(0,0,0) );
      global.obstacles.push_back( pipe );
      
      initPos = Pos(0,0,0.1);
      setCameraHomePos (Pos(2.90049, 11.4115, 2.62329),  Pos(169.839, -11.4999, 0));
      setCameraMode( Follow );
    }
    if( env == "slo" )
    {
      double dist = 10;
      double slope=1.2;
      double hight = 5.;
      double width = dist*1.4;
      OdeHandle wallHandle = odeHandle;
      wallHandle.substance.toPlastic(0.8);

      auto* wall1 = new Box(width,0.3,hight);
      wall1->init( wallHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      wall1->setSubstance(  Substance::getPlastic(0.8) );
      wall1->setPose( osg::Matrix::rotate(-slope,1.,0.,0.) * osg::Matrix::translate(0,dist/2.,0) );
      auto* wall2 = new Box(width,0.3,hight);
      wall2->init( wallHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      wall2->setSubstance(  Substance::getPlastic(0.8) );
      wall2->setPose( osg::Matrix::rotate(slope,1.,0.,0.) * osg::Matrix::translate(0,-dist/2.,0) );
      auto* wall3 = new Box(0.3,width,hight);
      wall3->init( wallHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      wall3->setSubstance(  Substance::getPlastic(0.8) );
      wall3->setPose( osg::Matrix::rotate(slope,0.,1.,0.) * osg::Matrix::translate(dist/2.,0.,0) );
      auto* wall4 = new Box(0.3,width,hight);
      wall4->init( wallHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      wall4->setSubstance(  Substance::getPlastic(0.8) );
      wall4->setPose( osg::Matrix::rotate(-slope,0.,1.,0.) * osg::Matrix::translate(-dist/2.,0.,0) );

      initPos = Pos(0,0,0);
      setCameraHomePos(Pos(-0.680767, -6.35182, 14.4558),  Pos(-2.99261, -66.8559, 0));
      setCameraMode( Static );
    }
    if( env == "slope" )
    {  /** 4 flat surfaces forming kind of a funnel */
      double slope = 0.3;
      auto* box = new Box(30,100,3);
      box->setTexture("Images/wood.jpg");
      box->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      box->setSubstance(Substance::getPlastic(0.8));
      box->setPose(osg::Matrix::rotate(slope,0.,1.,0.)); 
      auto* box_right = new Box(30,100,3);
      box_right->setTexture("Images/wood.jpg");
      box_right->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      box_right->setSubstance(Substance::getPlastic(0.8));
      box_right->setPose(osg::Matrix::rotate(slope,0.,-1.,0.)
                        *osg::Matrix::translate(40,0,0)); 
      auto* box_front = new Box(70,30,3);
      box_front->setTexture("Images/wood.jpg");
      box_front->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      box_front->setSubstance(Substance::getPlastic(0.8));
      box_front->setPose(osg::Matrix::rotate(slope,1.,0.,0.)
                        *osg::Matrix::translate(20,38,0)); 
      auto* box_back = new Box(70,30,3);
      box_back->setTexture("Images/wood.jpg");
      box_back->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      box_back->setSubstance(Substance::getPlastic(0.8));
      box_back->setPose(osg::Matrix::rotate(slope,-1.,0.,0.)
                        *osg::Matrix::translate(20,-38,0)); 
  
      //initPos = Pos(0,0,2.5);    
      initPos = Pos(-10,0,0);    
      setCameraHomePos(Pos(-5.51694, -11.5603, 6.56456),  Pos(-22.7518, -13.0505, 0));
      setCameraMode( Follow );
    }
    if( env == "playground" )
    { 
      OdeHandle wallHandle = odeHandle;
      //wallHandle.substance.toPlastic(0.8);
      wallHandle.substance.toRubber(25);
      OctaPlayground* world = new OctaPlayground( wallHandle, osgHandle, 
      									Pos(10,0.2,0.5), 6, false);
      world->setPose( osg::Matrix::translate(0,0,0) );
      global.obstacles.push_back( world );
      initPos = Pos(0,0,0);
      //setCameraHomePos(Pos(-0.671052,0 18.1675, 22.2259),  Pos(-179.845, -53.4133, 0));
      setCameraHomePos(Pos(20.9623, 0.306804, 11.942),  Pos(90.5044, -31.6968, 0));
      setCameraMode( Static );
      //setCameraHomePos (Pos(0.501969, 1.09391, 0.340535),  Pos(157.028, -10.8405, 0));
      //setCameraMode( Follow );
    }
    if( randObstacles ) {  	
      OdeHandle obstHandle = odeHandle;
      obstHandle.substance.toRubber(25);
      RandomObstaclesConf randConf = RandomObstacles::getDefaultConf();
      randConf.pose = osg::Matrix::translate(0,0,0);
      randConf.area = Pos(6,6,0.5);
      randConf.minSize = Pos(0.1,0.1,0.6);
      randConf.maxSize = Pos(1.,1.,0.6);
      randConf.minDensity = 2;
      randConf.maxDensity = 5;
      //randConf.area = Pos(5,5,2);
      //randConf.minSize = Pos(0.1,0.1,0.4);
      //randConf.maxSize = Pos(0.7,0.7,0.4);
      //randConf.minDensity = 2;
      //randConf.maxDensity = 5;
      RandomObstacles* RandObstacle = new RandomObstacles(obstHandle, osgHandle, randConf);
      if(randObstacles == true){  /** Generation and placing Objects */
        int num_randObs = 16;
        for (int i=0; i< num_randObs; i++){
          RandObstacle->spawn(RandomObstacles::Box, RandomObstacles::Foam);
          global.obstacles.push_back( RandObstacle );
        }
      }
    }
    if( env == "no" )
    {
      initPos = Pos(0,0,0);
      //setCameraHomePos(Pos(-2.24309, 0.920369, 1.45178),  Pos(-113.025, -29.863, 0));
      setCameraHomePos (Pos(0.501969, 1.09391, 0.340535),  Pos(157.028, -10.8405, 0));
      setCameraMode( Follow );
    }
    /********* END ENVIRONMENT ***********/
    
  
    /********* CAR CHAIN *****************/
    WheeledRobConf conf = WheeledRob::getDefaultConf();
    conf.carNumber     = carNum;
    conf.randomInitWP  = false;
    /** if only 1 car install support wheels */
    (conf.carNumber==1) ? conf.supportWheels=true : conf.supportWheels=false;
    auto robot = new WheeledRob( odeHandle, osgHandle, global.odeConfig, conf, "Car");
    robot->place( initPos );
    auto controller = new CouplingRod("CouplingRod", global.odeConfig);
    auto wiring = new One2OneWiring(new WhiteNormalNoise());
    auto agent = new OdeAgent(global);
    //auto agent = new OdeAgent( PlotOption(File) );
    agent->init(controller, robot, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);
    /** track options */
    TrackRobot* TrackOpt = new TrackRobot(false,false,false,true);
    TrackOpt->conf.displayTraceDur = 15;
    TrackOpt->conf.displayTraceThickness = 0.0; // 0.01 or 0.
    agent->setTrackOptions( *TrackOpt );
    /*********** END CAR CHAIN **********/
  }


  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(!pause) {
      OdeRobot* rob = globalData.agents[0]->getRobot();
      dynamic_cast<WheeledRob*>(rob)->velocityFriction( friction, friction_first_body ); 
    }
  }


  virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down) {
    if (down) {
      OdeRobot* rob = globalData.agents[0]->getRobot();
      switch( (char) key) { 
        case 'j': dBodyAddForce( rob->getMainPrimitive()->getBody(), 200, 0, 0); break;
        case 'J': dBodyAddForce( rob->getMainPrimitive()->getBody(), -200, 0, 0); break;
        case 'k': dBodyAddForce( rob->getMainPrimitive()->getBody(), 0, 200, 0); break;
        case 'K': dBodyAddForce( rob->getMainPrimitive()->getBody(), 0, -200, 0); break;
        case 'l': dBodyAddTorque( rob->getMainPrimitive()->getBody(), 0, 0, 5); break;
        case 'L': dBodyAddTorque( rob->getMainPrimitive()->getBody(), 0, 0, -5); break;
      }
    }
    return false;
  }


  virtual void bindingDescription(osg::ApplicationUsage & au) const { }
};




int main (int argc, char **argv)
{
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;
}
