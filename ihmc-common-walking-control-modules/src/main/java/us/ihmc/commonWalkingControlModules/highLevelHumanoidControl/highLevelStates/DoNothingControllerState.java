package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public class DoNothingControllerState extends HighLevelControllerState
{
   private static final HighLevelControllerName controllerState = HighLevelControllerName.DO_NOTHING_BEHAVIOR;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder;

   public DoNothingControllerState(OneDoFJointBasics[] controlledJoints, HighLevelControllerParameters highLevelControllerParameters)
   {
      this("", controlledJoints, highLevelControllerParameters);
   }

   public DoNothingControllerState(String namePrefix, OneDoFJointBasics[] controlledJoints, HighLevelControllerParameters highLevelControllerParameters)
   {
      super(namePrefix, controllerState, highLevelControllerParameters, controlledJoints);
      lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder(controlledJoints.length);
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);
   }

   @Override
   public void doAction(double timeInState)
   {
      for (int i = 0; i < controlledJoints.length; i++)
      {
         controlledJoints[i].setTau(0.0);
         lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(controlledJoints[i]).clear();
         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointTorque(controlledJoints[i], 0.0);
         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointPosition(controlledJoints[i], controlledJoints[i].getQ());
         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointVelocity(controlledJoints[i], controlledJoints[i].getQd());
      }

      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
   }

   @Override
   public void onEntry()
   {
      // Do nothing

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

   @Override
   public ControllerCoreOutput getControllerCoreOutput()
   {
      return null;
   }
   @Override
   public ControllerCoreCommand getControllerCoreCommandData()
   {
      return null;
   }
}
