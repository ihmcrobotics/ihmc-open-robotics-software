package us.ihmc.atlas.ObstacleCourseTests;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCWallWorldTest;

public class AtlasWallWorldTest extends DRCWallWorldTest
{
   private final AtlasRobotModel robotModel;
   
   public AtlasWallWorldTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS, false, false);
      
      AtlasContactPointParameters contactPointParameters = robotModel.getContactPointParameters();
      contactPointParameters.createHandKnobContactPoints();
   }
   
   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
}

