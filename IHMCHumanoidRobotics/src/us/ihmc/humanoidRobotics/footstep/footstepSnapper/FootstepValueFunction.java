package us.ihmc.humanoidRobotics.footstep.footstepSnapper;

import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;

/**
 * Created by agrabertilton on 1/19/15.
 */
public interface FootstepValueFunction
{
   public double getFootstepValue(FootstepDataMessage footstepData);
   public void updateFunction();
}
