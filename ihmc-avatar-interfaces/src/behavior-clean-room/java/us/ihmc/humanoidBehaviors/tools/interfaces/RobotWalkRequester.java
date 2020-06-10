package us.ihmc.humanoidBehaviors.tools.interfaces;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commons.thread.TypedNotification;

public interface RobotWalkRequester
{
   TypedNotification<WalkingStatusMessage> requestWalk(FootstepDataListMessage footstepDataListMessage);
}
