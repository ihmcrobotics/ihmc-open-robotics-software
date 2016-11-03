package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.humanoidRobotics.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.commonWalkingControlModules.controlModules.LockPelvisController;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCSteeringWheelEnvironment;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

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
