package us.ihmc.atlas;

import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.avatar.simulationStarter.DRCSimulationTools;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.simulationconstructionset.util.environments.PlanarRegionsListDefinedEnvironment;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class AtlasStandingOnPlanarRegionsListDefinedEnvironmentExample
{
   public static void main(String[] args)
   {
      AtlasRobotModel robotModel = AtlasRobotModelFactory.createDefaultRobotModel();

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      generator.addRectangle(20, 20);

      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment("FlatPlanarRegionsList", generator.getPlanarRegionsList(), 0.005, false);

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, environment);
      simulationStarter.setRunMultiThreaded(true);
      simulationStarter.setInitializeEstimatorToActual(true);
      DRCSimulationTools.startSimulationWithGraphicSelector(simulationStarter, null, null);
   }
}
