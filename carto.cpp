#include <iostream>
#include "Aria.h"



int main(int argc, char** argv)
{

//init
  Aria::init();
//actions def
  ArActionConstantVelocity Velocity("Constant Velocity", 1000);
  ArActionStallRecover recover;

  ArRobot robot;
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArRobotConnector robotConnector(&parser, &robot);
  ArLaserConnector laserConnector(&parser, &robot, &robotConnector);
  // Connect to the robot, get some initial data from it such as type and name,
  // and then load parameter files for this robot.
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "lasersExample: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        // -help not given
        Aria::logOptions();
        Aria::exit(1);
    }
  }
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(2);
    return 2;
  }

  ArLog::log(ArLog::Normal, "lasersExample: Connected to robot.");
  // Start the robot processing cycle running in the background.
  // True parameter means that if the connection is lost, then the
  // run loop ends.
  robot.runAsync(true);
//add Actions
  robot.addAction(&recover, 100);
  robot.addAction(&Velocity, 50);


  // Enable the motors, disable amigobot sounds
  robot.enableMotors();

  // Connect to laser(s) defined in parameter files, if LaseAutoConnect is true
  // for the laser.
  // (Some flags are available as arguments to connectLasers() to control error behavior and to control which lasers are put in the list of lasers stored by ArRobot. See docs for details.)
  ArLog::log(ArLog::Normal, "lasersExample: Connecting to any lasers enabled for connect...");
  if(!laserConnector.connectLasers())
  {
    ArLog::log(ArLog::Terse, "lasersExample: Could not connect to configured lasers. Exiting.");
    Aria::exit(3);
    return 3;
  }
  ArLog::log(ArLog::Normal, "lasersExample: Connected to all lasers.\n");
  // Log parameters related to the lasers

  const ArRobotParams *params = robot.getRobotParams();
  /*for(size_t i = 1; i <= robot.getNumLasers(); ++i)
  {
    if(!robot.findLaser(i))
      continue;
    ArLog::log(ArLog::Normal, "\tlaser #%d: pose=(x:%d, y:%d, z:%d, th:%0.2f), powerOutput=%s",
      i,
      params->getLaserX(i), params->getLaserY(i), params->getLaserZ(i), params->getLaserTh(i),
      params->getLaserPowerOutput(i)
    );
  }*/
  
  ArLog::log(ArLog::Normal, "lasersExample: ArLaserConnector also using the following parameters (if values given, otherwise defaults for this laser type will be used):");
  laserConnector.logLaserData();
  ArLog::log(ArLog::Normal, "");
  // Allow some time to read laser data
  ArUtil::sleep(500);
  // Print out some data from each connected laser.
  while(robot.isConnected())
  {
    int numLasers = 0;
      // Get a pointer to ArRobot's list of connected lasers. We will lock the robot while using it to prevent changes by tasks in the robot's background task thread or any other threads. Each laser has an index. You can also store the laser's index or name (laser->getName()) and use that to get a reference (pointer) to the laser object using ArRobot::findLaser().
      robot.lock();
      std::map<int, ArLaser*> *lasers = robot.getLaserMap();
    ArLog::log(ArLog::Normal, "lasersExample: ArRobot provided a set of %d ArLaser objects.", lasers->size());
      for(std::map<int, ArLaser*>::const_iterator i = lasers->begin(); i != lasers->end(); ++i)
      {
      int laserIndex = (*i).first;
      ArLaser* laser = (*i).second;
      if(!laser)
        continue;
      ++numLasers;
      laser->lockDevice();
      // The current readings are a set of obstacle readings (with X,Y positions as well as other attributes) that are the most recent set from teh laser.
      std::list<ArPoseWithTime*> *currentReadings = laser->getCurrentBuffer(); // see ArRangeDevice interface doc
      // The raw readings are just range or other data supplied by the sensor. It may also include some device-specific extra values associated with each reading as well. (e.g. Reflectance for LMS200)
      const std::list<ArSensorReading*> *rawReadings = laser->getRawReadings();

      // There is a utility to find the closest reading wthin a range of degrees around the laser, here we use this laser's full field of view (start to end)
      // If there are no valid closest readings within the given range, dist will be greater than laser->getMaxRange().
      double angle = 0;
      double dist = laser->currentReadingPolar(laser->getStartDegrees(), laser->getEndDegrees(), &angle);
      ArLog::log(ArLog::Normal, "lasersExample: Laser #%d (%s): %s.\n\tHave %d 'current' readings.\n\tHave %d 'raw' readings.\n\tClosest reading is at %3.0f degrees and is %2.4f meters away.",
        laserIndex, laser->getName(), (laser->isConnected() ? "connected" : "NOT CONNECTED"),
        currentReadings->size(),
        rawReadings->size(),
        angle, dist/1000.0);
                  laser->unlockDevice();
    }
    if(numLasers == 0)
      ArLog::log(ArLog::Normal, "lasersExample: No lasers.");
    else
      ArLog::log(ArLog::Normal, "");
    // Unlock robot and sleep for a few seconds before next loop.
    robot.unlock();
    ArUtil::sleep(3000);
  }
  ArLog::log(ArLog::Normal, "lasersExample: exiting.");


  // Start the robot processing cycle running in the background.
  // True parameter means that if the connection is lost, then the
  // run loop ends.


  robot.waitForRunExit();
  Aria::shutdown();
  return 0;
}
