package us.ihmc.valkyrie.roughTerrainWalking;

import java.io.InputStream;
import java.util.Objects;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.HumanoidEndToEndSlopeTest;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyrieEndToEndSlopeTest extends HumanoidEndToEndSlopeTest
{
   private static final String STEEP_SLOPES_PARAMETERS_XML = "/us/ihmc/valkyrie/simulation/steep_slopes_parameters.xml";

   private boolean useVal2Scale = false;
   private boolean removeAnkleJointLimits = false;

   private boolean goUp = true;
   private double swingDuration = 0.6;
   private double transferDuration = 0.25;
   private double maxStepLength = 0.30;
   private double heightOffset = 0.0;
   private double torsoPitch = 0.0;
   private boolean useSideSteps = false;
   private boolean disableToeOff = false;

   @Override
   public DRCRobotModel getRobotModel()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS)
      {
         @Override
         public InputStream getParameterOverwrites()
         {
            InputStream resourceAsStream = ValkyrieEndToEndSlopeTest.class.getResourceAsStream(STEEP_SLOPES_PARAMETERS_XML);
            Objects.requireNonNull(resourceAsStream);
            return resourceAsStream;
         }
      };

      if (useVal2Scale)
      {
         robotModel.setModelSizeScale(0.925170);
         robotModel.setModelMassScale(0.925170);
      }

      if (removeAnkleJointLimits)
      {
         robotModel.setRobotDefinitionMutator(robotModel.getRobotDefinitionMutator()
                                                        .andThen(RobotDefinitionTools.jointLimitMutator("Ankle", -Math.PI, Math.PI)));
      }

      return robotModel;
   }

   @BeforeEach
   public void initializeTest()
   {
      useVal2Scale = false;
      removeAnkleJointLimits = false;

      goUp = true;
      swingDuration = 0.6;
      transferDuration = 0.25;
      maxStepLength = 0.30;
      heightOffset = 0.0;
      torsoPitch = 0.0;
      useSideSteps = false;
   }

   @Test
   @Disabled // Quite redundant with the Val2Scale tests.
   @Tag("humanoid-rough-terrain-slow")
   public void testUpSlope(TestInfo testInfo) throws Exception
   {
      testSlope(testInfo, goUp, useSideSteps, swingDuration, transferDuration, maxStepLength, heightOffset, torsoPitch, false, disableToeOff);
   }

   @Test
   @Disabled // Quite redundant with the Val2Scale tests.
   @Tag("humanoid-rough-terrain-slow")
   public void testUpSlopeExperimentalPhysicsEngine(TestInfo testInfo) throws Exception
   {
      swingDuration = 0.8;
      transferDuration = 0.35;
      maxStepLength = 0.20;
      heightOffset = 0.025;
      torsoPitch = 0.666;
      testSlope(testInfo, goUp, useSideSteps, swingDuration, transferDuration, maxStepLength, heightOffset, torsoPitch, true, disableToeOff);
   }

   @Test
   @Disabled // Quite redundant with the Val2Scale tests.
   @Tag("humanoid-rough-terrain-slow")
   public void testDownSlope(TestInfo testInfo) throws Exception
   {
      goUp = false;
      swingDuration = 0.9;
      maxStepLength = 0.25;
      testSlope(testInfo, goUp, useSideSteps, swingDuration, transferDuration, maxStepLength, heightOffset, torsoPitch, false, disableToeOff);
   }

   @Test
   @Disabled // Quite redundant with the Val2Scale tests.
   @Tag("humanoid-rough-terrain-slow")
   public void testDownSlopeExperimentalPhysicsEngine(TestInfo testInfo) throws Exception
   {
      goUp = false;
      swingDuration = 1.0;
      transferDuration = 0.5;
      maxStepLength = 0.40;
      testSlope(testInfo, goUp, useSideSteps, swingDuration, transferDuration, maxStepLength, heightOffset, torsoPitch, true, disableToeOff);
   }

   @Test
   @Tag("humanoid-rough-terrain-slow")
   public void testUpSlopeVal2Scale(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      maxStepLength = 0.25;
      torsoPitch = 0.666;
      testSlope(testInfo, goUp, useSideSteps, swingDuration, transferDuration, maxStepLength, heightOffset, torsoPitch, false, disableToeOff);
   }

   @Test
   @Tag("humanoid-rough-terrain-slow")
   public void testUpSlopeVal2ScaleExperimentalPhysicsEngine(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      swingDuration = 0.8;
      transferDuration = 0.40;
      maxStepLength = 0.20;
      heightOffset = 0.04;
      torsoPitch = 0.666;
      testSlope(testInfo, goUp, useSideSteps, swingDuration, transferDuration, maxStepLength, heightOffset, torsoPitch, true, disableToeOff);
   }

   @Test
   @Tag("humanoid-rough-terrain-slow")
   public void testDownSlopeVal2Scale(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      goUp = false;
      swingDuration = 1.0;
      transferDuration = 0.35;
      testSlope(testInfo, goUp, useSideSteps, swingDuration, transferDuration, maxStepLength, heightOffset, torsoPitch, false, disableToeOff);
   }

   @Test
   @Tag("humanoid-rough-terrain-slow")
   public void testDownSlopeVal2ScaleExperimentalPhysicsEngine(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      goUp = false;
      swingDuration = 1.0;
      transferDuration = 0.5;
      maxStepLength = 0.35;
      testSlope(testInfo, goUp, useSideSteps, swingDuration, transferDuration, maxStepLength, heightOffset, torsoPitch, true, disableToeOff);
   }

   @Test
   @Tag("humanoid-rough-terrain-slow")
   public void testUpSlopeVal2ScaleExperimentalPhysicsEngineNoLimits(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      removeAnkleJointLimits = true;
      maxStepLength = 0.25;
      testSlope(testInfo, goUp, useSideSteps, swingDuration, transferDuration, maxStepLength, heightOffset, torsoPitch, true, disableToeOff);
   }

   @Test
   @Tag("humanoid-rough-terrain-slow")
   public void testUpSlopeVal2ScaleExperimentalPhysicsEngineNoLimitsSideSteps(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      removeAnkleJointLimits = true;
      useSideSteps = true;
      swingDuration = 0.8;
      transferDuration = 0.5;
      maxStepLength = 0.25;
      disableToeOff = true;
      testSlope(testInfo, goUp, useSideSteps, swingDuration, transferDuration, maxStepLength, heightOffset, torsoPitch, true, disableToeOff);
   }

   @Test
   @Tag("humanoid-rough-terrain-slow")
   public void testDownSlopeVal2ScaleExperimentalPhysicsEngineNoLimitsSideSteps(TestInfo testInfo) throws Exception
   {
      goUp = false;
      useVal2Scale = true;
      removeAnkleJointLimits = true;
      useSideSteps = true;
      swingDuration = 0.8;
      transferDuration = 0.5;
      maxStepLength = 0.20;
      disableToeOff = true;
      testSlope(testInfo, goUp, useSideSteps, swingDuration, transferDuration, maxStepLength, heightOffset, torsoPitch, true, disableToeOff);
   }
}
