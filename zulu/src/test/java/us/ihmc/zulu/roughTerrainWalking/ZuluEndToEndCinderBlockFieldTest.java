package us.ihmc.zulu.roughTerrainWalking;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.EndToEndCinderBlockFieldTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class ZuluEndToEndCinderBlockFieldTest extends EndToEndCinderBlockFieldTest
{
   private final DRCRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public double getPelvisOffsetHeight()
   {
      return -0.0;
   }

   @Override
   public double getStepHeightOffset()
   {
      return 0.0;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ZULU);
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
