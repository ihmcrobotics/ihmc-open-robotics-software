package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.JumpStateEnum;
import us.ihmc.yoVariables.providers.EnumProvider;
import us.ihmc.yoVariables.variable.YoEnum;

public interface JumpControlManagerInterface
{
   InverseDynamicsCommand<?> getInverseDynamicsCommand();
   FeedbackControlCommand<?> getFeedbackControlCommand();
   void setStateEnumProvider(EnumProvider<JumpStateEnum> stateEnumProvider);
   void compute();
}
