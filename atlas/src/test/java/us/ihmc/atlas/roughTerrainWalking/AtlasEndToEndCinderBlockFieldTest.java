package us.ihmc.atlas.roughTerrainWalking;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.EndToEndCinderBlockFieldTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class AtlasEndToEndCinderBlockFieldTest extends EndToEndCinderBlockFieldTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public double getPelvisOffsetHeight()
   {
      return -0.05;
   }

   @Override
   public double getStepHeightOffset()
   {
      return 0.0;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @Tag("humanoid-rough-terrain")
   @Test
   public void testWalkingOverCinderBlockField() throws Exception
   {
      super.testWalkingOverCinderBlockField();
   }

   @Override
   @Tag("humanoid-rough-terrain-slow")
   @Test
   public void testSteppingStonesA() throws Exception
   {
      super.testSteppingStonesA();
   }

   @Tag("humanoid-rough-terrain-slow")
   @Test
   public void testSlantedCinderBlockFieldA() throws Exception
   {
      super.testSlantedCinderBlockField(false);
   }
}
