package us.ihmc.zulu.roughTerrainWalking;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarSwingOverPlanarRegionsTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-rough-terrain")
public class ZuluSwingOverPlanarRegionsTest extends AvatarSwingOverPlanarRegionsTest
{
   @Override
   @Disabled // This test is not applicable to Zulu, TODO fix me
   @Test
   public void testSwingOverPlanarRegions()
   {
      super.testSwingOverPlanarRegions();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ZULU);
   }
}
