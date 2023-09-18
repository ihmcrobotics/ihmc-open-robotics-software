package us.ihmc.valkyrie.roughTerrainWalking;

import java.io.InputStream;
import java.util.Objects;
import java.util.Random;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.HumanoidEndToEndStairsTest;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.simulation.ValkyrieFlatGroundQuickWalkingTest;

@Tag("humanoid-stairs-slow")
public class ValkyrieEndToEndStairsTest extends HumanoidEndToEndStairsTest
{
   private static final String OSHA_DOWNSTAIRS_PARAMETERS_XML = "/us/ihmc/valkyrie/simulation/fast_walking_parameters.xml";

   private boolean useVal2Scale = false;
   private boolean useCustomDownstairsParameters = false;

   @Override
   public DRCRobotModel getRobotModel()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS)
      {
         @Override
         public InputStream getParameterOverwrites()
         {
            if (useCustomDownstairsParameters)
            {
               InputStream resourceAsStream = ValkyrieFlatGroundQuickWalkingTest.class.getResourceAsStream(OSHA_DOWNSTAIRS_PARAMETERS_XML);
               Objects.requireNonNull(resourceAsStream);
               return resourceAsStream;
            }
            else
            {
               return null;
            }
         }
      };
      if (useVal2Scale)
         robotModel.setVal2Scale();
      return robotModel;
   }

   @BeforeEach
   public void initializeTest()
   {
      useVal2Scale = false;
      useCustomDownstairsParameters = false;
   }

   @Test
   public void testUpStairsSlow(TestInfo testInfo) throws Exception
   {
      Random random = new Random(53415);
      testStairs(testInfo, true, true, 0.6, 0.25, 0.0, createFootstepCorruptor(random, 0.025, 0.10, 0.05, 0.2, 0.2, 0.2));
   }

   @Test
   public void testDownStairsSlow(TestInfo testInfo) throws Exception
   {
      Random random = new Random(53415);
      testStairs(testInfo, true, false, 0.9, 0.25, 0.0, createFootstepCorruptor(random, 0.025, 0.06, 0.025, 0.2, 0.2, 0.2));
   }

   @Test
   public void testUpStairs(TestInfo testInfo) throws Exception
   {
      testStairs(testInfo, false, true, 1.0, 0.25, 0.025);
   }

   @Test
   public void testDownStairs(TestInfo testInfo) throws Exception
   {
      Random random = new Random(53415);
      testStairs(testInfo, false, false, 1.4, 0.35, 0.0, createFootstepCorruptor(random, 0.025, 0.10, 0.05, 0.2, 0.2, 0.2));
   }

   @Test
   public void testUpStairsSlowVal2Scale(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      Random random = new Random(53415);
      testStairs(testInfo, true, true, 0.6, 0.25, 0.0, createFootstepCorruptor(random, 0.02, 0.05, 0.05, 0.2, 0.2, 0.2));
   }

   @Test
   public void testDownStairsSlowVal2Scale(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      Random random = new Random(53415);
      testStairs(testInfo, true, false, 1.0, 0.25, 0.0, createFootstepCorruptor(random, 0.01, 0.05, 0.05, 0.1, 0.2, 0.2));
   }

   @Test
   public void testUpStairsVal2Scale(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      testStairs(testInfo, false, true, 1.2, 0.25, 0.0);
   }

   @Test
   public void testDownStairsVal2Scale(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      Random random = new Random(53415);
      testStairs(testInfo, false, false, 1.0, 0.35, 0.0, createFootstepCorruptor(random, 0.02, 0.05, 0.05, 0.2, 0.2, 0.2));
   }

   @Test
   public void testUpStairsSlowVal2ScaleExperimentalPhysicsEngine(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      setUseExperimentalPhysicsEngine(true);
      DRCRobotModel robotModel = getRobotModel();
      double footLength = robotModel.getWalkingControllerParameters().getSteppingParameters().getFootLength();
      double footWidth = robotModel.getWalkingControllerParameters().getSteppingParameters().getFootWidth();
      double leftFootOffsetX = 0.05;

      testStairs(testInfo, true, true, 0.6, 0.25, 0.04, footsteps ->
      {
         int numberOfSteps = footsteps.getFootstepDataList().size();

         for (int i = 0; i < numberOfSteps; i++)
         {
            FootstepDataMessage footstep = footsteps.getFootstepDataList().get(i);
            footstep.getLocation().subX(0.05);

            if (i > 1 && i < numberOfSteps - 2 && footstep.getRobotSide() == RobotSide.LEFT.toByte())
            {
               footstep.getLocation().subX(leftFootOffsetX);
               footstep.getPredictedContactPoints2d().add().set(0.5 * footLength, 0.5 * footWidth, 0);
               footstep.getPredictedContactPoints2d().add().set(0.5 * footLength, -0.5 * footWidth, 0);
               footstep.getPredictedContactPoints2d().add().set(-0.5 * footLength + leftFootOffsetX, 0.5 * footWidth, 0);
               footstep.getPredictedContactPoints2d().add().set(-0.5 * footLength + leftFootOffsetX, -0.5 * footWidth, 0);
            }
         }
      });
   }

   @Test
   public void testDownStairsSlowVal2ScaleExperimentalPhysicsEngine(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      useCustomDownstairsParameters = true;
      setUseExperimentalPhysicsEngine(true);
      DRCRobotModel robotModel = getRobotModel();
      double footLength = robotModel.getWalkingControllerParameters().getSteppingParameters().getFootLength();
      double footWidth = robotModel.getWalkingControllerParameters().getSteppingParameters().getFootWidth();
      double rightFootOffsetX = 0.05;

      testStairs(testInfo, true, false, 0.8, 0.25, 0.0, footsteps ->
      {
         int numberOfSteps = footsteps.getFootstepDataList().size();

         for (int i = 0; i < numberOfSteps; i++)
         {
            FootstepDataMessage footstep = footsteps.getFootstepDataList().get(i);

            if (footstep.getRobotSide() == RobotSide.RIGHT.toByte() && i != numberOfSteps - 1)
            {
               footstep.getLocation().addX(0.035);
               footstep.getLocation().addX(rightFootOffsetX);
               footstep.getPredictedContactPoints2d().add().set(0.5 * footLength - rightFootOffsetX, 0.5 * footWidth, 0);
               footstep.getPredictedContactPoints2d().add().set(0.5 * footLength - rightFootOffsetX, -0.5 * footWidth, 0);
               footstep.getPredictedContactPoints2d().add().set(-0.5 * footLength, 0.5 * footWidth, 0);
               footstep.getPredictedContactPoints2d().add().set(-0.5 * footLength, -0.5 * footWidth, 0);
            }
            else
            {
               footstep.getLocation().subX(0.01);
            }
         }
      });
   }

   @Test
   public void testUpStairsVal2ScaleExperimentalPhysicsEngine(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      setUseExperimentalPhysicsEngine(true);
      DRCRobotModel robotModel = getRobotModel();
      double footLength = robotModel.getWalkingControllerParameters().getSteppingParameters().getFootLength();
      double footWidth = robotModel.getWalkingControllerParameters().getSteppingParameters().getFootWidth();
      double footOffsetX = 0.10;

      testStairs(testInfo, false, true, 1.2, 0.25, 0.0, footsteps ->
      {
         int numberOfSteps = footsteps.getFootstepDataList().size();
         for (int i = 0; i < numberOfSteps; i++)
         {
            FootstepDataMessage footstep = footsteps.getFootstepDataList().get(i);

            if (i > 1 && i < numberOfSteps - 2)
            {
               footstep.getLocation().subX(0.05);
               footstep.getLocation().subX(footOffsetX);
               footstep.getPredictedContactPoints2d().add().set(0.5 * footLength, 0.5 * footWidth, 0);
               footstep.getPredictedContactPoints2d().add().set(0.5 * footLength, -0.5 * footWidth, 0);
               footstep.getPredictedContactPoints2d().add().set(-0.5 * footLength + footOffsetX, 0.5 * footWidth, 0);
               footstep.getPredictedContactPoints2d().add().set(-0.5 * footLength + footOffsetX, -0.5 * footWidth, 0);
            }
         }
      });
   }

   @Test
   public void testDownStairsVal2ScaleExperimentalPhysicsEngine(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      useCustomDownstairsParameters = true;
      setUseExperimentalPhysicsEngine(true);
      DRCRobotModel robotModel = getRobotModel();
      double footLength = robotModel.getWalkingControllerParameters().getSteppingParameters().getFootLength();
      double footWidth = robotModel.getWalkingControllerParameters().getSteppingParameters().getFootWidth();
      double footOffsetX = 0.025;

      testStairs(testInfo, false, false, 1.2, 0.25, 0.0, footsteps ->
      {
         int numberOfSteps = footsteps.getFootstepDataList().size();
         for (int i = 0; i < numberOfSteps; i++)
         {
            FootstepDataMessage footstep = footsteps.getFootstepDataList().get(i);

            if (i < numberOfSteps - 2)
            {
               footstep.getLocation().addX(0.035);
               footstep.getLocation().addX(footOffsetX);
               footstep.getPredictedContactPoints2d().add().set(0.5 * footLength - footOffsetX, 0.5 * footWidth, 0);
               footstep.getPredictedContactPoints2d().add().set(0.5 * footLength - footOffsetX, -0.5 * footWidth, 0);
               footstep.getPredictedContactPoints2d().add().set(-0.5 * footLength, 0.5 * footWidth, 0);
               footstep.getPredictedContactPoints2d().add().set(-0.5 * footLength, -0.5 * footWidth, 0);
            }
         }
      });
   }
}
