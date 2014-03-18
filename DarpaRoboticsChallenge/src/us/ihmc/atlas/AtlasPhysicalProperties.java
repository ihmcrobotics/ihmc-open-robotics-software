package us.ihmc.atlas;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;

public class AtlasPhysicalProperties extends DRCRobotPhysicalProperties
{
   private final double ankleHeight = 0.084;
   @Override
   public double getAnkleHeight()
   {
      return ankleHeight;
   }

}
