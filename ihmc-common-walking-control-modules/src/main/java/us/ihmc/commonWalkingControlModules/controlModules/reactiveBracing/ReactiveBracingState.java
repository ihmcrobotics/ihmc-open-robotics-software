package us.ihmc.commonWalkingControlModules.controlModules.reactiveBracing;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;

public interface ReactiveBracingState extends State
{
   InverseDynamicsCommand<?> getInverseDynamicsCommand();

   FeedbackControlCommand<?> getFeedbackControlCommand();

   FeedbackControlCommand<?> createFeedbackControlTemplate();

   YoGraphicDefinition getSCS2YoGraphics();
}
