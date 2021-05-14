package us.ihmc.atlas.roughTerrainWalking;

import java.io.InputStream;
import java.util.Objects;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.HumanoidEndToEndSlopeTest;

public class AtlasEndToEndSlopeTest extends HumanoidEndToEndSlopeTest
{
   private static final String STEEP_SLOPES_PARAMETERS_XML = "/us/ihmc/atlas/steep_slopes_parameters.xml";

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
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS)
      {
         @Override
         public InputStream getParameterOverwrites()
         {
            InputStream resourceAsStream = AtlasEndToEndSlopeTest.class.getResourceAsStream(STEEP_SLOPES_PARAMETERS_XML);
            Objects.requireNonNull(resourceAsStream);
            return resourceAsStream;
         }
      };
   }

   @BeforeEach
   public void initializeTest()
   {
      goUp = true;
      swingDuration = 0.6;
      transferDuration = 0.25;
      maxStepLength = 0.30;
      heightOffset = 0.0;
      torsoPitch = 0.0;
      useSideSteps = false;
   }

   @Test
   @Tag("humanoid-rough-terrain-slow")
   public void testUpSlopeExperimentalPhysicsEngine(TestInfo testInfo) throws Exception
   {
      maxStepLength = 0.25;
      heightOffset = 0.0;
      torsoPitch = 0.50;
      testSlope(testInfo, goUp, useSideSteps, swingDuration, transferDuration, maxStepLength, heightOffset, torsoPitch, true, disableToeOff);
   }

   @Test
   @Tag("humanoid-rough-terrain-slow")
   public void testDownSlopeExperimentalPhysicsEngine(TestInfo testInfo) throws Exception
   {
      goUp = false;
      swingDuration = 1.0;
      transferDuration = 0.5;
      maxStepLength = 0.30;
      testSlope(testInfo, goUp, useSideSteps, swingDuration, transferDuration, maxStepLength, heightOffset, torsoPitch, true, disableToeOff);
   }
}
