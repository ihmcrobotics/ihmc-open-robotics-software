package us.ihmc.atlas.behaviorTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.DRCHeadTrajectoryBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-behaviors-slow")
public class AtlasHeadTrajectoryBehaviorTest extends DRCHeadTrajectoryBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasHeadTrajectoryBehaviorTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @Test
   public void testHeadPitch()
   {
      super.testHeadPitch();
   }

   @Override
   @Disabled
   @Test
   public void testHeadRoll()
   {
      super.testHeadRoll();
   }

   @Override
   @Disabled
   @Test
   public void testHeadYaw()
   {
      super.testHeadYaw();
   }

   @Override
   @Disabled
   @Test
   public void testRandomOrientation()
   {
      super.testRandomOrientation();
   }

}
