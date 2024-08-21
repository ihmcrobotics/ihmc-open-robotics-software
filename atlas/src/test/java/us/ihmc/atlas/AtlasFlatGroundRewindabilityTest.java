package us.ihmc.atlas;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.DRCFlatGroundRewindabilityTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-flat-ground")
@Disabled
public class AtlasFlatGroundRewindabilityTest extends DRCFlatGroundRewindabilityTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @Test
   public void testCanRewindAndGoForward()
   {
      super.testCanRewindAndGoForward();
   }

   @Override
   @Disabled
   @Test
   public void testRewindabilityWithSimpleFastMethod()
   {
      super.testRewindabilityWithSimpleFastMethod();
   }

   @Override
   // This takes a long time. Use it for debugging where the broken changes were made when the tests above fail.
   @Disabled
   @Test
   public void testRewindabilityWithSlowerMoreExtensiveMethod()
   {
      super.testRewindabilityWithSlowerMoreExtensiveMethod();
   }

   @Override
   @Test
   public void testRunsTheSameWayTwice()
   {
      super.testRunsTheSameWayTwice();
   }

}
