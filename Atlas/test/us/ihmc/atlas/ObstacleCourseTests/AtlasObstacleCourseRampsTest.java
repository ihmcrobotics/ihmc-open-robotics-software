package us.ihmc.atlas.ObstacleCourseTests;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseRampsTest;

public class AtlasObstacleCourseRampsTest extends DRCObstacleCourseRampsTest
{
   private DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS, false, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   protected double getMaxRotationCorruption()
   {
      return Math.PI/8.0;
   }

}
