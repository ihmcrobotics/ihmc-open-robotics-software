package us.ihmc.humanoidRobotics.footstep.footstepSnapper;

import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepData;

/**
 * Created by agrabertilton on 1/19/15.
 */
public interface FootstepValueFunction
{
   public double getFootstepValue(FootstepData footstepData);
   public void updateFunction();
}
