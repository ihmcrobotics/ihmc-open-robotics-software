package us.ihmc.acsell.simulation;

import us.ihmc.acsell.operatorInterface.StepprOperatorInterface;
import us.ihmc.acsell.parameters.BonoRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseDemoStarter;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.utilities.processManagement.JavaProcessSpawner;

public class StepprObstacleCourseDemo extends DRCObstacleCourseDemoStarter
{
   private static final DRCRobotModel robotModel = new BonoRobotModel(false, false);

   public static void main(String[] args)
   {
      StepprObstacleCourseDemo stepprObstacleCourseDemo = new StepprObstacleCourseDemo();
      boolean automaticallyStartSimulation = true;
      boolean startDRCNetworkProcessor = true; // set to false to use slider board instead.
      boolean initializeEstimatorToActual = false;
      
      CommonAvatarEnvironmentInterface environment = new DRCDemo01NavigationEnvironment();
      stepprObstacleCourseDemo.obstacleCourseStarter(environment, robotModel, initializeEstimatorToActual, automaticallyStartSimulation, startDRCNetworkProcessor);
   }

   @Override
   public void SpawnUI(DRCRobotModel robotModel)
   {
      JavaProcessSpawner spawner = new JavaProcessSpawner(true);
      String[] args = {};
      spawner.spawn(StepprOperatorInterface.class, args);
   }
}
