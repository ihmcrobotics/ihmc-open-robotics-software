package us.ihmc.atlas.controllerAPI;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndEndFootBearingMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

public class AtlasEndToEndFootLoadBearingTest extends EndToEndEndFootBearingMessageTest
{

   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);

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
