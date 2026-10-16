package us.ihmc.zulu.roughTerrainWalking;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.HumanoidEndToEndSlopeTest;

public class ZuluEndToEndSlopeTest extends HumanoidEndToEndSlopeTest
{
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
      return new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
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
      swingDuration = 1.0;
      transferDuration = 0.5;
      maxStepLength = 0.25;
      heightOffset = 0.0;
      torsoPitch = 0.50;
      disableToeOff = true;
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
