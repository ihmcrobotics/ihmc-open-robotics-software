package us.ihmc.atlas.rrtWalkingpathtest;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;

public class AtlasWalkingPathGeneratorTest extends AvatarWalkingPathGeneratorTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      // TODO Auto-generated method stub
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      // TODO Auto-generated method stub
      return "RobotName";
   }
}
