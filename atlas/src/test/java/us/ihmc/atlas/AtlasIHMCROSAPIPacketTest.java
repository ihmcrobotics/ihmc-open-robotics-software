package us.ihmc.atlas;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.IHMCROSAPIPacketTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;


@Disabled
public class AtlasIHMCROSAPIPacketTest extends IHMCROSAPIPacketTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @Test
   public void testFuzzyPacketsUsingRos()
   {
      super.testFuzzyPacketsUsingRos();
   }

   @Override
   @Test
   public void testFuzzyPacketsWithoutRos()
   {
      super.testFuzzyPacketsWithoutRos();
   }

}
