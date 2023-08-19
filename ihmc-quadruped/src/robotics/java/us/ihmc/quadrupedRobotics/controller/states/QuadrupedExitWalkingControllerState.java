package us.ihmc.quadrupedRobotics.controller.states;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HoldPositionControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.SmoothTransitionControllerState;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class QuadrupedExitWalkingControllerState extends SmoothTransitionControllerState
{
   private static final String namePrefix = "exitWalking";

   private final HoldPositionControllerState finalControllerState;

   public QuadrupedExitWalkingControllerState(HighLevelControllerState initialControllerState,
                                              HoldPositionControllerState finalControllerState,
                                              OneDoFJointBasics[] controlledJoints,
                                              HighLevelControllerParameters highLevelControllerParameters,
                                              CommandInputManager commandInputManager)
   {
      super(namePrefix, HighLevelControllerName.EXIT_WALKING, initialControllerState, finalControllerState, controlledJoints, highLevelControllerParameters, commandInputManager);

      this.finalControllerState = finalControllerState;
   }

   @Override
   public void onEntry()
   {
      super.onEntry();

      finalControllerState.setToCurrent();
   }

}
