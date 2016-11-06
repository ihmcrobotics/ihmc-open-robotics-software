package us.ihmc.atlas;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.darpaRoboticsChallenge.simulationStarter.DRCSimulationStarter;
import us.ihmc.simulationconstructionset.util.environments.DefaultCommonAvatarEnvironment;

public class AtlasAutomaticDiagnosticDemo
{
   public AtlasAutomaticDiagnosticDemo()
   {
      DRCRobotModel robotModel = AtlasRobotModelFactory.createDefaultRobotModel();

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, new DefaultCommonAvatarEnvironment());
      simulationStarter.setRunMultiThreaded(true);
      //      simulationStarter.registerHighLevelController(new CarIngressEgressControllerFactory(robotModel.getMultiContactControllerParameters(), true));

      DRCNetworkModuleParameters networkProcessorParameters;
      networkProcessorParameters = new DRCNetworkModuleParameters();
      networkProcessorParameters.enableBehaviorModule(true);
      networkProcessorParameters.enableBehaviorVisualizer(true);
      networkProcessorParameters.enableAutomaticDiagnostic(true, 15);

      boolean automaticallyStartSimulation = true;
      simulationStarter.startSimulation(networkProcessorParameters, automaticallyStartSimulation);

   }

   public static void main(String[] args)
   {
      new AtlasAutomaticDiagnosticDemo();
   }
}
