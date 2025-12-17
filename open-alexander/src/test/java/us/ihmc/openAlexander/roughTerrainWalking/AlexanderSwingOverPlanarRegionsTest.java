package us.ihmc.openAlexander.roughTerrainWalking;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarSwingOverPlanarRegionsTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-rough-terrain")
public class AlexanderSwingOverPlanarRegionsTest extends AvatarSwingOverPlanarRegionsTest
{
   @Override
   @Disabled // This test is not applicable to Alexander V0, as it works with V1 version
   @Test
   public void testSwingOverPlanarRegions()
   {
      super.testSwingOverPlanarRegions();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }
}
