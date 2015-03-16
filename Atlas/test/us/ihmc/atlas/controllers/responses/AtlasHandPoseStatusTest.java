package us.ihmc.atlas.controllers.responses;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.controllerResponse.HandPoseStatusTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;

public class AtlasHandPoseStatusTest extends HandPoseStatusTest
{
   private final AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS_UNPLUGGED_V4, AtlasRobotModel.AtlasTarget.SIM, false);
   
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
