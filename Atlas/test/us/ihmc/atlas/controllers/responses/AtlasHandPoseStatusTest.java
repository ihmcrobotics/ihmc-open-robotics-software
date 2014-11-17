package us.ihmc.atlas.controllers.responses;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.controllerResponse.HandPoseStatusTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

public class AtlasHandPoseStatusTest extends HandPoseStatusTest
{
private AtlasRobotModel robotModel= new AtlasRobotModel(AtlasRobotVersion.ATLAS_ROBOTIQ_HOOK, AtlasRobotModel.AtlasTarget.SIM, false); 
   
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
