package us.ihmc.openAlexander;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.avatar.simulationStarter.DRCSimulationTools;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;

public class AlexanderStandingOnPlanarRegionsListDefinedEnvironmentExample
{
   public static void main(String[] args)
   {
      OpenAlexanderRobotModel robotModel = new OpenAlexanderRobotModel(OpenAlexanderVersion.V1_FULL_ROBOT, RobotTarget.SCS);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      generator.addRectangle(20, 20);

      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment("FlatPlanarRegionsList", generator.getPlanarRegionsList(), 0.005, false);

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, environment);
      simulationStarter.setRunMultiThreaded(true);
      simulationStarter.setInitializeEstimatorToActual(true);
      DRCSimulationTools.startSimulationWithGraphicSelector(simulationStarter, null, null);
   }
}
