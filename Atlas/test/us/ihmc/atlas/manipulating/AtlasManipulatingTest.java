package us.ihmc.atlas.manipulating;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel.RobotTarget;
import us.ihmc.avatar.manipulating.DRCManipulatingReachingTest;

public class AtlasManipulatingTest extends DRCManipulatingReachingTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      // TODO Auto-generated method stub
      //return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false);
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      // TODO Auto-generated method stub
      return "RobotName";
   }

}
