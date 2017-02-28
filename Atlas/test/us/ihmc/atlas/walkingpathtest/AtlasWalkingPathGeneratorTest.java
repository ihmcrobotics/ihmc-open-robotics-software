package us.ihmc.atlas.walkingpathtest;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel.RobotTarget;
import us.ihmc.avatar.rrtwalkingpath.AvatarWalkingPathGeneratorTest;

public class AtlasWalkingPathGeneratorTest extends AvatarWalkingPathGeneratorTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      // TODO Auto-generated method stub
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      // TODO Auto-generated method stub
      return "RobotName";
   }

}
