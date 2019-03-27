package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.humanoidBehaviors.BehaviorBackpack;
import us.ihmc.log.LogTools;

public class AtlasBehaviorBackpack
{
   public static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;

   public AtlasBehaviorBackpack()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false);

      LogTools.info("Creating behavior module");
      BehaviorBackpack.createForBackpack(robotModel);
   }

   /** To run remotely */
   public static void main(String[] args)
   {
      new AtlasBehaviorBackpack();
   }
}
