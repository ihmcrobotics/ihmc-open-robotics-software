package us.ihmc.humanoidRobotics.footstep.footstepSnapper;

import controller_msgs.msg.dds.FootstepDataMessage;

/**
 * Created by agrabertilton on 1/19/15.
 */
public interface FootstepValueFunction
{
   public double getFootstepValue(FootstepDataMessage footstepData);
   public void updateFunction();
}
