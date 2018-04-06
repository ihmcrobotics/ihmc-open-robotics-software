package us.ihmc.quadrupedRobotics.output;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.OutputWriter;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.LowLevelStateList;
import us.ihmc.simulationToolkit.controllers.LowLevelActuatorSimulator;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.HashMap;

public class SimulatedQuadrupedOutputWriter implements OutputWriter
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final OneDoFJoint[] controllerJoints;

   private final HashMap<OneDoFJoint, QuadrupedJoint> quadrupedJoints = new HashMap<>();
   private final HashMap<OneDoFJoint, LowLevelActuatorSimulator> quadrupedActuators = new HashMap<>();

   public SimulatedQuadrupedOutputWriter(FloatingRootJointRobot robot, FullRobotModel fullRobotModel, JointDesiredOutputListReadOnly jointDesiredOutputList,
                                         double controlDT)
   {
      controllerJoints = fullRobotModel.getOneDoFJoints();
      LowLevelStateList lowLevelStateList = new LowLevelStateList(controllerJoints);

      for (OneDoFJoint controllerJoint : controllerJoints)
      {
         quadrupedJoints.put(controllerJoint, new QuadrupedJoint(lowLevelStateList.getLowLevelState(controllerJoint), controllerJoint,
                                                                 jointDesiredOutputList.getJointDesiredOutput(controllerJoint), controlDT, registry));

         String name = controllerJoint.getName();
         OneDegreeOfFreedomJoint simulatedJoint = robot.getOneDegreeOfFreedomJoint(name);
         LowLevelActuatorSimulator actuator = new LowLevelActuatorSimulator(simulatedJoint, lowLevelStateList.getLowLevelState(controllerJoint), controlDT);
         quadrupedActuators.put(controllerJoint, actuator);
         robot.setController(actuator);
      }

      robot.getRobotsYoVariableRegistry().addChild(registry);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      throw new RuntimeException("This should have been done earlier.");
   }

   @Override
   public void write()
   {
      for (OneDoFJoint controllerJoint : controllerJoints)
      {
         quadrupedJoints.get(controllerJoint).computeDesiredStateFromJointController();
         quadrupedActuators.get(controllerJoint).setActuatorMode(quadrupedJoints.get(controllerJoint).getDesiredActuatorMode());
      }
   }
}
