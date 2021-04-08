package us.ihmc.atlas;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.AvatarRangeOfMotionTests;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

public class AtlasRangeOfMotionTests extends AvatarRangeOfMotionTests
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public double getDesiredPelvisHeightAboveFoot()
   {
      return 0.4;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testSquattingDown() throws Exception
   {
      super.testSquattingDown();
   }

}
