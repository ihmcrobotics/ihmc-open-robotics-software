package us.ihmc.quadrupedRobotics.controller.states;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HoldPositionControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointControlBlender;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.SmoothTransitionControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedExitWalkingControllerState extends SmoothTransitionControllerState
{
   private static final String namePrefix = "exitWalking";

   private final HoldPositionControllerState finalControllerState;

   public QuadrupedExitWalkingControllerState(HighLevelControllerState initialControllerState, HoldPositionControllerState finalControllerState,
                                              OneDoFJointBasics[] controlledJoints, HighLevelControllerParameters highLevelControllerParameters)
   {
      super(namePrefix, HighLevelControllerName.EXIT_WALKING, initialControllerState, finalControllerState, controlledJoints, highLevelControllerParameters);

      this.finalControllerState = finalControllerState;
   }

   @Override
   public void onEntry()
   {
      super.onEntry();

      finalControllerState.setToCurrent();
   }

}
