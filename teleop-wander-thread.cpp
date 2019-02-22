
#include "Aria.h"

ArActionGroup *teleop;
ArActionGroup *wander;

pthread_t teleop_thread;
pthread_t wander_thread;
pthread_mutex_t mutex;

// Activate the teleop action group. activateExlcusive() causes
// all other active action groups to be deactivated.
void teleopMode(void)
{
  teleop->activateExclusive();
  printf("\n== Teleoperation Mode ==\n");
  printf("   Use the arrow keys to drive, and the spacebar to stop.\n    For joystick control hold the trigger button.\n    Press 'w' to switch to wander mode.\n    Press escape to exit.\n");
}

// Activate the wander action group. activateExlcusive() causes
// all other active action groups to be deactivated.
void wanderMode(void)
{
  wander->activateExclusive();
  printf("\n== Wander Mode ==\n");
  printf("    The robot will now just wander around avoiding things.\n    Press 't' to switch to  teleop mode.\n    Press escape to exit.\n");
}

void *teleopInit(void *input){
	pthread_mutex_lock(&mutex);
	
	teleopMode();
	
	pthread_mutex_unlock(&mutex);
	return NULL;
}

void *wanderInit(void *input){
	pthread_mutex_lock(&mutex);
	wanderMode();
	pthread_mutex_unlock(&mutex);
	return NULL;
}


int main(int argc, char** argv)
{
  Aria::init();
  ArArgumentParser argParser(&argc, argv);
  ArRobot robot;
  ArRobotConnector con(&argParser, &robot);
  ArSonarDevice sonar;

  argParser.addDefaultArgument("-rh 10.0.126.18");
  argParser.loadDefaultArguments();
  if(!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    return 1;
  }

  /* - the action group for teleoperation actions: */
  teleop = new ArActionGroup(&robot);

  // don't hit any tables (if the robot has IR table sensors)
  teleop->addAction(new ArActionLimiterTableSensor, 100);

  // limiter for close obstacles
  teleop->addAction(new ArActionLimiterForwards("speed limiter near", 
						300, 600, 250), 95);

  // limiter for far away obstacles
  teleop->addAction(new ArActionLimiterForwards("speed limiter far", 
					       300, 1100, 400), 90);

  // limiter so we don't bump things backwards
  teleop->addAction(new ArActionLimiterBackwards, 85);

  // the joydrive action (drive from joystick)
  ArActionJoydrive joydriveAct("joydrive", 400, 15);
  teleop->addAction(&joydriveAct, 50);

  // the keydrive action (drive from keyboard)
  teleop->addAction(new ArActionKeydrive, 45);
  


  /* - the action group for wander actions: */
  wander = new ArActionGroup(&robot);

  // if we're stalled we want to back up and recover
  wander->addAction(new ArActionStallRecover, 100);

  // react to bumpers
  wander->addAction(new ArActionBumpers, 75);

  // turn to avoid things closer to us
  wander->addAction(new ArActionAvoidFront("Avoid Front Near", 225, 0), 50);

  // turn avoid things further away
  wander->addAction(new ArActionAvoidFront, 45);

  // keep moving
  wander->addAction(new ArActionConstantVelocity("Constant Velocity", 400), 25);

  

  /* - use key commands to switch modes, and use keyboard
   *   and joystick as inputs for teleoperation actions. */

  // create key handler if Aria does not already have one
  ArKeyHandler *keyHandler = Aria::getKeyHandler();
  if (keyHandler == NULL)
  {
    keyHandler = new ArKeyHandler;
    Aria::setKeyHandler(keyHandler);
    robot.attachKeyHandler(keyHandler);
  }

  // set the callbacks
  ArGlobalFunctor teleopCB(&teleopMode);
  ArGlobalFunctor wanderCB(&wanderMode);
  keyHandler->addKeyHandler('w', &wanderCB);
  keyHandler->addKeyHandler('W', &wanderCB);
  keyHandler->addKeyHandler('t', &teleopCB);
  keyHandler->addKeyHandler('T', &teleopCB);

  // if we don't have a joystick, let 'em know
  if (!joydriveAct.joystickInited())
    printf("Note: Do not have a joystick, only the arrow keys on the keyboard will work.\n");
  
  // set the joystick so it won't do anything if the button isn't pressed
  joydriveAct.setStopIfNoButtonPressed(false);


  /* - connect to the robot, then enter teleoperation mode.  */

  robot.addRangeDevice(&sonar);
  if(!con.connectRobot(&robot))
  { 
    ArLog::log(ArLog::Terse, "actionGroupExample: Could not connect to the robot.");
    Aria::exit(1);
  }

  robot.enableMotors();
  pthread_create(&teleop_thread, NULL, teleopInit, NULL);
  pthread_create(&wander_thread, NULL, wanderInit, NULL);
  
  robot.run(true);
  
  pthread_join(teleop_thread, NULL);
  pthread_join(wander_thread, NULL);

  Aria::exit(0);
}
