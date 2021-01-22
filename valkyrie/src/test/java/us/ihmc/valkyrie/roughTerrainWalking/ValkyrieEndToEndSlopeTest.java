package us.ihmc.valkyrie.roughTerrainWalking;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.HumanoidEndToEndSlopeTest;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyrieEndToEndSlopeTest extends HumanoidEndToEndSlopeTest
{
   private boolean useVal2Scale = false;

   @Override
   public DRCRobotModel getRobotModel()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      if (useVal2Scale)
      {
         robotModel.setModelSizeScale(0.925170);
         robotModel.setModelMassScale(0.925170);
      }
      return robotModel;
   }

   @Test
   public void testUpSlope(TestInfo testInfo) throws Exception
   {
      useVal2Scale = false;
      super.testSlope(testInfo, true, 0.6, 0.25, 0.4, 0.0);
   }

   @Test
   public void testDownSlope(TestInfo testInfo) throws Exception
   {
      useVal2Scale = false;
      super.testSlope(testInfo, false, 0.8, 0.35, 0.3, 0.0);
   }

   @Test
   public void testUpSlopeVal2Scale(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      super.testSlope(testInfo, true, 0.8, 0.25, 0.25, 0.0);
   }

   @Test
   public void testDownSlopeVal2Scale(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      super.testSlope(testInfo, false, 0.8, 0.3, 0.30, 0.0);
   }
}
