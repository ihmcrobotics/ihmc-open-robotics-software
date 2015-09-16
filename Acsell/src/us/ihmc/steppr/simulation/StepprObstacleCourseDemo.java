package us.ihmc.steppr.simulation;

import us.ihmc.darpaRoboticsChallenge.DRCSimulationStarter;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationTools;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.steppr.parameters.BonoRobotModel;

public class StepprObstacleCourseDemo
{
   public static void main(String[] args)
   {
      DRCRobotModel robotModel = new BonoRobotModel(false, false);
      DRCSimulationStarter createDRCDemo01SimulationStarter = new DRCSimulationStarter(robotModel, new DRCDemo01NavigationEnvironment());
      createDRCDemo01SimulationStarter.setRunMultiThreaded(true);
      
      DRCSimulationTools.startSimulationWithGraphicSelector(createDRCDemo01SimulationStarter);
   }
}
