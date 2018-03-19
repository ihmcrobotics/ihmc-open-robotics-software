package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedStepTransitionCallback;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedWaypointCallback;
import us.ihmc.robotics.stateMachine.old.eventBasedStateMachine.FiniteStateMachineState;

public abstract class QuadrupedFootState implements FiniteStateMachineState<QuadrupedFootControlModule.FootEvent>
{
   protected final FrameVector3D soleForceCommand = new FrameVector3D();

   protected QuadrupedStepTransitionCallback stepTransitionCallback = null;
   protected QuadrupedWaypointCallback waypointCallback = null;

   public abstract SpatialFeedbackControlCommand getFeedbackControlCommand();

   public FrameVector3DReadOnly getSoleForceCommand()
   {
      return soleForceCommand;
   }

   public void registerStepTransitionCallback(QuadrupedStepTransitionCallback stepTransitionCallback)
   {
      this.stepTransitionCallback = stepTransitionCallback;
   }

   public void registerWaypointCallback(QuadrupedWaypointCallback waypointCallback)
   {
      this.waypointCallback = waypointCallback;
   }
}
