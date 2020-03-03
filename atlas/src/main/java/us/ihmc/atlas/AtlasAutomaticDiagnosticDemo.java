package us.ihmc.atlas;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessorParameters;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;

public class AtlasAutomaticDiagnosticDemo
{
   public AtlasAutomaticDiagnosticDemo()
   {
      DRCRobotModel robotModel = AtlasRobotModelFactory.createDefaultRobotModel();

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, new DefaultCommonAvatarEnvironment());
      simulationStarter.setRunMultiThreaded(true);
      //      simulationStarter.registerHighLevelController(new CarIngressEgressControllerFactory(robotModel.getMultiContactControllerParameters(), true));

      HumanoidNetworkProcessorParameters networkProcessorParameters;
      networkProcessorParameters = new HumanoidNetworkProcessorParameters();
      networkProcessorParameters.setUseAutomaticDiagnostic(true, true, 15);

      boolean automaticallyStartSimulation = true;
      simulationStarter.startSimulation(networkProcessorParameters, automaticallyStartSimulation);

   }

   public static void main(String[] args)
   {
      new AtlasAutomaticDiagnosticDemo();
   }
}
