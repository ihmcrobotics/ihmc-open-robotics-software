package us.ihmc.atlas.controllers;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.DRCLocalConfigParameters;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCRobotBasedFootstepGeneratorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

public class AtlasFootstepGeneratorTest extends DRCRobotBasedFootstepGeneratorTest
{
   private static final AtlasRobotVersion ATLAS_ROBOT_VERSION = AtlasRobotVersion.DRC_NO_HANDS;
   private static final boolean RUNNING_ON_REAL_ROBOT = DRCLocalConfigParameters.RUNNING_ON_REAL_ROBOT;
   private final static DRCRobotModel robotModel = new AtlasRobotModel(ATLAS_ROBOT_VERSION, RUNNING_ON_REAL_ROBOT);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

}
