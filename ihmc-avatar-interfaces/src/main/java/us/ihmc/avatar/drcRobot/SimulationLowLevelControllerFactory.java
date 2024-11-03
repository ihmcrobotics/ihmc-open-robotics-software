package us.ihmc.avatar.drcRobot;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.RobotController;

public interface SimulationLowLevelControllerFactory
{
   RobotController createLowLevelController(FullRobotModel controllerRobot, Robot simulatedRobot, JointDesiredOutputListReadOnly controllerDesiredOutputList);
}
