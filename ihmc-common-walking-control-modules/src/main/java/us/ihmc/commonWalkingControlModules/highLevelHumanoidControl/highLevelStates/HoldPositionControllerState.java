package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.commons.robotics.outputData.JointDesiredOutputBasics;
import us.ihmc.commons.robotics.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.commons.robotics.outputData.JointDesiredOutputReadOnly;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.variable.YoDouble;

public class HoldPositionControllerState extends HighLevelControllerState
{
   private final JointDesiredOutputListReadOnly highLevelControllerOutput;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final PairList<OneDoFJointBasics, YoDouble> jointSetpoints = new PairList<>();

   public HoldPositionControllerState(HighLevelControllerName stateEnum, OneDoFJointBasics[] controlledJoints,
                                      HighLevelControllerParameters highLevelControllerParameters, JointDesiredOutputListReadOnly highLevelControllerOutput)
   {
      super(stateEnum, highLevelControllerParameters, controlledJoints);

      this.highLevelControllerOutput = highLevelControllerOutput;
      String nameSuffix = "_" + stateEnum.name();

      for (OneDoFJointBasics controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();

         YoDouble freezePosition = new YoDouble(jointName + nameSuffix + "_qDesired", registry);
         freezePosition.setToNaN();

         jointSetpoints.add(controlledJoint, freezePosition);
      }

      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);
   }

   public void setToCurrent()
   {
      for (int jointIndex = 0; jointIndex < jointSetpoints.size(); jointIndex++)
      {
         OneDoFJointBasics joint = jointSetpoints.get(jointIndex).getLeft();
         YoDouble setpoint = jointSetpoints.get(jointIndex).getRight();
         setpoint.set(joint.getQ());
      }
   }

   @Override
   public void onEntry()
   {
      for (int jointIndex = 0; jointIndex < jointSetpoints.size(); jointIndex++)
      {
         OneDoFJointBasics joint = jointSetpoints.get(jointIndex).getLeft();
         YoDouble setpoint = jointSetpoints.get(jointIndex).getRight();
         JointDesiredOutputReadOnly lowLevelJointData = highLevelControllerOutput.getJointDesiredOutput(joint);
         if (lowLevelJointData != null && lowLevelJointData.hasDesiredPosition())
            setpoint.set(lowLevelJointData.getDesiredPosition());
         else
            setpoint.set(joint.getQ());
      }
   }

   @Override
   public void doAction(double timeInState)
   {
      for (int jointIndex = 0; jointIndex < jointSetpoints.size(); jointIndex++)
      {
         OneDoFJointBasics joint = jointSetpoints.get(jointIndex).getLeft();
         YoDouble desiredPosition = jointSetpoints.get(jointIndex).getRight();

         JointDesiredOutputBasics lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
         lowLevelJointData.clear();
         lowLevelJointData.setDesiredPosition(desiredPosition.getDoubleValue());
         lowLevelJointData.setDesiredVelocity(0.0);
         lowLevelJointData.setDesiredAcceleration(0.0);
      }

      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
   }

   @Override
   public void onExit(double timeInState)
   {
      // Do nothing

   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }
}
