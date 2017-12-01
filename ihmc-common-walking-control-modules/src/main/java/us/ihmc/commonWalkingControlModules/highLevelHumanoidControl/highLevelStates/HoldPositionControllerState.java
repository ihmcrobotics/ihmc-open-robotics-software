package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class HoldPositionControllerState extends HighLevelControllerState
{
   private final YoVariableRegistry registry;

   private final JointDesiredOutputListReadOnly highLevelControllerOutput;
   protected final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final PairList<OneDoFJoint, YoDouble> jointSetpoints = new PairList<>();

   public HoldPositionControllerState(HighLevelControllerName stateEnum, HighLevelHumanoidControllerToolbox controllerToolbox,
                                      JointDesiredOutputListReadOnly highLevelControllerOutput)
   {
      super(stateEnum);

      this.highLevelControllerOutput = highLevelControllerOutput;
      String nameSuffix = stateEnum.name();
      registry = new YoVariableRegistry(nameSuffix + getClass().getSimpleName());
      nameSuffix = "_" + nameSuffix;

      OneDoFJoint[] controlledJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();

      for (OneDoFJoint controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();

         YoDouble freezePosition = new YoDouble(jointName + nameSuffix + "_qDesired", registry);
         freezePosition.setToNaN();

         jointSetpoints.add(controlledJoint, freezePosition);
      }

      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (int jointIndex = 0; jointIndex < jointSetpoints.size(); jointIndex++)
      {
         OneDoFJoint joint = jointSetpoints.get(jointIndex).getLeft();
         YoDouble setpoint = jointSetpoints.get(jointIndex).getRight();
         JointDesiredOutputReadOnly lowLevelJointData = highLevelControllerOutput.getJointDesiredOutput(joint);
         if (lowLevelJointData != null && lowLevelJointData.hasDesiredPosition())
            setpoint.set(lowLevelJointData.getDesiredPosition());
         else
            setpoint.set(joint.getQ());
      }
   }

   @Override
   public void doAction()
   {
      for (int jointIndex = 0; jointIndex < jointSetpoints.size(); jointIndex++)
      {
         OneDoFJoint joint = jointSetpoints.get(jointIndex).getLeft();
         YoDouble desiredPosition = jointSetpoints.get(jointIndex).getRight();

         JointDesiredOutput lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
         lowLevelJointData.setDesiredPosition(desiredPosition.getDoubleValue());
         lowLevelJointData.setDesiredVelocity(0.0);
         lowLevelJointData.setDesiredAcceleration(0.0);
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // Do nothing

   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }
}
