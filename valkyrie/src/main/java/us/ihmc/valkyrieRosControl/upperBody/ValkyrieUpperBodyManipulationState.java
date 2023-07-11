package us.ihmc.valkyrieRosControl.upperBody;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public class ValkyrieUpperBodyManipulationState extends HighLevelControllerState
{
   private static final HighLevelControllerName controllerName = HighLevelControllerName.WALKING;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   public ValkyrieUpperBodyManipulationState(HighLevelControllerParameters parameters, OneDoFJointBasics[] controlledJoints)
   {
      super(controllerName, parameters, controlledJoints);
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   @Override
   public void onEntry()
   {

   }

   @Override
   public void doAction(double timeInState)
   {

   }

   @Override
   public void onExit(double timeInState)
   {

   }
}
