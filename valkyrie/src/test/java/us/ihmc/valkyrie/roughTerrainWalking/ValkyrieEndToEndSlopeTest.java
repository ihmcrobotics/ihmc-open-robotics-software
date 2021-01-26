package us.ihmc.valkyrie.roughTerrainWalking;

import java.io.InputStream;
import java.util.Objects;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.HumanoidEndToEndSlopeTest;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.simulation.ValkyrieFlatGroundFastWalkingTest;

public class ValkyrieEndToEndSlopeTest extends HumanoidEndToEndSlopeTest
{
   private static final String STEEP_SLOPES_PARAMETERS_XML = "/us/ihmc/valkyrie/simulation/steep_slopes_parameters.xml";

   private boolean useVal2Scale = false;

   @Override
   public DRCRobotModel getRobotModel()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS)
      {
         @Override
         public InputStream getParameterOverwrites()
         {
            InputStream resourceAsStream = ValkyrieFlatGroundFastWalkingTest.class.getResourceAsStream(STEEP_SLOPES_PARAMETERS_XML);
            Objects.requireNonNull(resourceAsStream);
            return resourceAsStream;
         }
      };
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
      super.testSlope(testInfo, true, 0.6, 0.25, 0.4, 0.0, 0.0, false);
   }

   @Test
   public void testUpSlopeExperimentalPhysicsEngine(TestInfo testInfo) throws Exception
   {
      useVal2Scale = false;
      super.testSlope(testInfo, true, 0.6, 0.25, 0.20, 0.0, Math.toRadians(45.0), true);
   }

   @Test
   public void testDownSlope(TestInfo testInfo) throws Exception
   {
      useVal2Scale = false;
      super.testSlope(testInfo, false, 0.9, 0.25, 0.25, 0.0, 0.0, false);
   }

   @Test
   public void testDownSlopeExperimentalPhysicsEngine(TestInfo testInfo) throws Exception
   {
      useVal2Scale = false;
      super.testSlope(testInfo, false, 1.2, 0.5, 0.25, 0.0, 0.0, true);
   }

   @Test
   public void testUpSlopeVal2Scale(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      super.testSlope(testInfo, true, 0.8, 0.25, 0.25, 0.0, 0.0, false);
   }

   @Test
   public void testUpSlopeVal2ScaleExperimentalPhysicsEngine(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      super.testSlope(testInfo, true, 0.6, 0.25, 0.15, 0.0, Math.toRadians(45.0), true);
   }

   @Test
   public void testDownSlopeVal2Scale(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      super.testSlope(testInfo, false, 0.8, 0.3, 0.30, 0.0, 0.0, false);
   }

   @Test
   public void testDownSlopeVal2ScaleExperimentalPhysicsEngine(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      super.testSlope(testInfo, false, 1.2, 0.5, 0.25, 0.0, 0.0, true);
   }
}
