package us.ihmc.humanoidBehaviors.tools.interfaces;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface RobotWalkRequester
{
   TypedNotification<WalkingStatusMessage> requestWalk(FootstepDataListMessage footstepPlan,
                                                              HumanoidReferenceFrames humanoidReferenceFrames,
                                                              PlanarRegionsList planarRegionsList);
}
