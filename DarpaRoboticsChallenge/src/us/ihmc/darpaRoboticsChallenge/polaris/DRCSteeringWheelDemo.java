package us.ihmc.darpaRoboticsChallenge.polaris;

import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.DRCSteeringWheelEnvironment;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.darpaRoboticsChallenge.simulationStarter.DRCSimulationStarter;
import us.ihmc.simulationToolkit.controllers.LockPelvisController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;

public class DRCSteeringWheelDemo
{
   private final DRCSimulationFactory drcSimulationFactory;

   public DRCSteeringWheelDemo(DRCRobotModel model, DRCNetworkModuleParameters networkParameters, DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup, double
         percentOfSteeringWheelRadius)
   {
      CommonAvatarEnvironmentInterface environment = new DRCSteeringWheelEnvironment(percentOfSteeringWheelRadius);

      DRCSimulationStarter simStarter = new DRCSimulationStarter(model, environment);

      simStarter.setRunMultiThreaded(true);      
      simStarter.setUsePerfectSensors(true);
      simStarter.setRobotInitialSetup(initialSetup);
      simStarter.setInitializeEstimatorToActual(true);
      simStarter.startSimulation(networkParameters, true);
      
      drcSimulationFactory = simStarter.getDRCSimulationFactory();
      FloatingRootJointRobot robot = drcSimulationFactory.getRobot();
      
      LockPelvisController controller = new LockPelvisController(robot, drcSimulationFactory.getSimulationConstructionSet(), model.createFullRobotModel(), 0.0);
      robot.setController(controller);
      controller.initialize();
      
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return drcSimulationFactory.getSimulationConstructionSet();
   }
}
