package us.ihmc.darpaRoboticsChallenge.wholeBodyInverseKinematicsSimulationController;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.CarIngressEgressControllerFactory;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationFactory;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationStarter;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.FlatGroundEnvironment;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class WholeBodyIKIngressEgressControllerSimulation
{
   private final SimulationConstructionSet simulationConstructionSet;
   private final DRCSimulationFactory drcSimulation;
//   private final YoVariableRegistry registry = new YoVariableRegistry("hik");
//   private DoubleYoVariable hik_arm_pos_x_d;
//   private DoubleYoVariable hik_arm_pos_y_d;
//   private DoubleYoVariable hik_arm_pos_z_d;

   public WholeBodyIKIngressEgressControllerSimulation(DRCRobotModel robotModel)
   {
      WalkingControllerParameters multiContactControllerParameters = robotModel.getMultiContactControllerParameters();
      FlatGroundEnvironment environment = new FlatGroundEnvironment();

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, environment);
      simulationStarter.setRunMultiThreaded(true);

      simulationStarter.registerHighLevelController(new CarIngressEgressControllerFactory(multiContactControllerParameters, false));
      simulationStarter.setInitializeEstimatorToActual(true);
      
      boolean automaticallyStartSimulation = true;
      DRCNetworkModuleParameters networkProcessorParameters = null; //new DRCNetworkModuleParameters();
//      networkProcessorParameters.setUseUiModule(false);
//      networkProcessorParameters.setUseBehaviorModule(false);
//      networkProcessorParameters.setUsePerceptionModule(false);
//      networkProcessorParameters.setUseSensorModule(false);
      

      simulationStarter.startSimulation(networkProcessorParameters, automaticallyStartSimulation);

      drcSimulation = simulationStarter.getDRCSimulationFactory();
      simulationConstructionSet = simulationStarter.getSimulationConstructionSet();
   }

   public DRCSimulationFactory getDRCSimulation()
   {
      return drcSimulation;
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return simulationConstructionSet;
   }

}
