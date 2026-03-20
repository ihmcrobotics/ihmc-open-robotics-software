package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import controller_msgs.msg.dds.DirectionalControlInputMessage;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ControllerWaypointGoalCommand;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;

public interface VelocityBasedGoalReacher
{
   void consumeNewWaypoint(ControllerWaypointGoalCommand command);

   void setCurrentGoalPose(FramePose2DReadOnly currentGoalPose);

   DirectionalControlInputMessage compute();

   DirectionalControlInputMessage compute(FramePose3DReadOnly currentPose);

   FramePose3DReadOnly getCurrentPose();

   FramePose2DReadOnly getCurrentGoalPose();

   DirectionalControlInputMessage getOutputMessage();
}
