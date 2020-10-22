package us.ihmc.valkyrie.simulation;

import us.ihmc.avatar.drcRobot.SimulationLowLevelControllerFactory;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class ValkyrieSimulationLowLevelControllerFactory implements SimulationLowLevelControllerFactory
{
   private final double controlDT;
   private final DRCRobotJointMap jointMap;

   public ValkyrieSimulationLowLevelControllerFactory(DRCRobotJointMap jointMap, double controlDT)
   {
      this.jointMap = jointMap;
      this.controlDT = controlDT;
   }

   @Override
   public RobotController createLowLevelController(FullRobotModel controllerRobot, Robot simulatedRobot,
                                                   JointDesiredOutputListReadOnly controllerDesiredOutputList)
   {
      ValkyrieSimulationLowLevelController controller = new ValkyrieSimulationLowLevelController(controllerRobot,
                                                                                                 simulatedRobot,
                                                                                                 controllerDesiredOutputList,
                                                                                                 controlDT);
      controller.addJointControllers(jointMap.getPositionControlledJointsForSimulation());
      return controller;
   }
}
