package us.ihmc.acsell.simulation;

import us.ihmc.acsell.parameters.BonoRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationStarter;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationTools;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

public class StepprObstacleCourseDemo
{
   public static void main(String[] args)
   {
      DRCRobotModel robotModel = new BonoRobotModel(false, false);
      boolean runMultiThreaded = true;
      DRCSimulationStarter createDRCDemo01SimulationStarter = DRCSimulationTools.createObstacleCourseSimulationStarter(runMultiThreaded, robotModel);
      DRCSimulationTools.startSimulationWithGraphicSelector(createDRCDemo01SimulationStarter);
   }
}
