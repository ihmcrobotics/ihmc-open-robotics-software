package us.ihmc.atlas.behaviorTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.DRCObjectWeightBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-behaviors-slow")
public class AtlasObjectWeightBehaviorTest extends DRCObjectWeightBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasObjectWeightBehaviorTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   @Test
   public void testConstructorAndSetInput()
   {
      super.testConstructorAndSetInput();
   }

   @Override
   @Disabled("Needs to be reimplemented")
   @Test
   public void testSettingWeight()
   {
      super.testSettingWeight();
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ATLAS);
   }
}
