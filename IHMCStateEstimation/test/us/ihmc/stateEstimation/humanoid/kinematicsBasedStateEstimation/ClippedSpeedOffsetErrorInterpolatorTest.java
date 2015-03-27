package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.code.agileTesting.BambooPlanType;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

@BambooPlan(planType={BambooPlanType.Fast})
public class ClippedSpeedOffsetErrorInterpolatorTest
{
   SimulationTestingParameters simulationTestingParameters = new SimulationTestingParameters();

   SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
   SimulationConstructionSet scs;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable alphaFilterBreakFrequency = new DoubleYoVariable("alphaFilterBreakFrequency", registry);
   private final double dt = 0.001;

   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void showMemoryAfterTests()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @EstimatedDuration(duration = 1.0)
   @Test(timeout = 60000)
   public void testRandomTranslationErrorInterpolation()
   {
      Random random = new Random();
      int numberOfTicks = 2000;
      alphaFilterBreakFrequency.set(0.6);

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, worldFrame,
            alphaFilterBreakFrequency, dt, false);

      for (int i = 0; i < 1000; i++)
      {
         FramePose startPose = new FramePose(worldFrame);
         FramePose goalPose = new FramePose(worldFrame);
         FramePose interpolatedOffset = new FramePose(worldFrame);

         startPose.setToZero(worldFrame);
         goalPose.setPose(RandomTools.generateRandomVector(random, 0.08), new Quat4d());

         clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startPose, goalPose, 1.0);

         for (int j = 0; j < numberOfTicks; j++)
         {
            clippedSpeedOffsetErrorInterpolator.interpolateError(interpolatedOffset);
         }
         assertTrue(interpolatedOffset.epsilonEquals(goalPose, 1e-4));
      }
   }

   @EstimatedDuration(duration = 2.0)
   @Test(timeout = 60000)
   public void testRandomRotationErrorInterpolation()
   {
      Random random = new Random();
      int numberOfTicks = 2000;
      alphaFilterBreakFrequency.set(0.6);

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, worldFrame,
            alphaFilterBreakFrequency, dt, true);

      for (int i = 0; i < 1000; i++)
      {
         FramePose startPose = new FramePose(worldFrame);
         FramePose goalPose = new FramePose(worldFrame);
         FramePose interpolatedOffset = new FramePose(worldFrame);

         startPose.setToZero(worldFrame);
         goalPose.setPose(new Vector3d(), RandomTools.generateRandomQuaternion(random, 0.08));

         clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startPose, goalPose, 1.0);

         for (int j = 0; j < numberOfTicks; j++)
         {
            clippedSpeedOffsetErrorInterpolator.interpolateError(interpolatedOffset);
         }
         assertTrue(interpolatedOffset.epsilonEquals(goalPose, 1e-4));
      }
   }

   @EstimatedDuration(duration = 2.0)
   @Test(timeout = 60000)
   public void testTranslationAndRotationErrorsInterpolation()
   {
      Random random = new Random();
      int numberOfTicks = 2000;
      alphaFilterBreakFrequency.set(0.6);

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, worldFrame,
            alphaFilterBreakFrequency, dt, true);

      for (int i = 0; i < 1000; i++)
      {
         FramePose startPose = new FramePose(worldFrame);
         FramePose goalPose = new FramePose(worldFrame);
         FramePose interpolatedOffset = new FramePose(worldFrame);

         startPose.setToZero(worldFrame);
         goalPose.setPose(RandomTools.generateRandomVector(random, 0.08), RandomTools.generateRandomQuaternion(random, 0.08));

         clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startPose, goalPose, 1.0);

         for (int j = 0; j < numberOfTicks; j++)
         {
            clippedSpeedOffsetErrorInterpolator.interpolateError(interpolatedOffset);
         }
         assertTrue(interpolatedOffset.epsilonEquals(goalPose, 1e-4));
      }
   }

   @EstimatedDuration(duration = 0.5)
   @Test(timeout = 60000)
   public void testMaxTranslationalCorrectionSpeedClip()
   {
      int numberOfTicks = 2000;
      alphaFilterBreakFrequency.set(0.6);

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, worldFrame,
            alphaFilterBreakFrequency, dt, false);

      FramePose startPose = new FramePose(worldFrame);
      FramePose goalPose = new FramePose(worldFrame);
      FramePose interpolatedOffset = new FramePose(worldFrame);
      FramePose interpolatedOffsetAfterOneSecond = new FramePose(worldFrame);

      startPose.setToZero(worldFrame);
      goalPose.setPose(new Vector3d(1.0, 0.0, 0.0), new Quat4d());

      clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startPose, goalPose, 1.0);

      for (int j = 0; j < numberOfTicks; j++)
      {
         clippedSpeedOffsetErrorInterpolator.interpolateError(interpolatedOffset);

         if (j == 999)
            interpolatedOffsetAfterOneSecond.setPose(interpolatedOffset);
      }

      assertTrue(Math.abs(startPose.getX() - interpolatedOffsetAfterOneSecond.getX()) <= 0.05);
      assertTrue(Math.abs(startPose.getY() - interpolatedOffsetAfterOneSecond.getY()) <= 0.05);
      assertTrue(Math.abs(startPose.getZ() - interpolatedOffsetAfterOneSecond.getZ()) <= 0.05);
      assertTrue(Math.abs(startPose.getYaw() - interpolatedOffsetAfterOneSecond.getYaw()) <= 0.05);
      assertTrue(Math.abs(startPose.getPitch() - interpolatedOffsetAfterOneSecond.getPitch()) <= 0.05);
      assertTrue(Math.abs(startPose.getRoll() - interpolatedOffsetAfterOneSecond.getRoll()) <= 0.05);
   }

   @EstimatedDuration(duration = 0.1)
   @Test(timeout = 60000)
   public void testMaxRotationalCorrectionSpeedClip()
   {
      int numberOfTicks = 2000;
      alphaFilterBreakFrequency.set(0.6);

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, worldFrame,
            alphaFilterBreakFrequency, dt, true);

      FramePose startPose = new FramePose(worldFrame);
      FramePose goalPose = new FramePose(worldFrame);
      FramePose interpolatedOffset = new FramePose(worldFrame);
      FramePose interpolatedOffsetAfterOneSecond = new FramePose(worldFrame);

      startPose.setToZero(worldFrame);
      Quat4d rotation = new Quat4d();
      RotationFunctions.setQuaternionBasedOnYawPitchRoll(rotation, 1.0, 0.0, 0.0);
      goalPose.setPose(new Vector3d(), rotation);

      clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startPose, goalPose, 1.0);

      for (int j = 0; j < numberOfTicks; j++)
      {
         clippedSpeedOffsetErrorInterpolator.interpolateError(interpolatedOffset);

         if (j == 999)
            interpolatedOffsetAfterOneSecond.setPose(interpolatedOffset);
      }

      assertTrue(Math.abs(startPose.getX() - interpolatedOffsetAfterOneSecond.getX()) <= 0.05);
      assertTrue(Math.abs(startPose.getY() - interpolatedOffsetAfterOneSecond.getY()) <= 0.05);
      assertTrue(Math.abs(startPose.getZ() - interpolatedOffsetAfterOneSecond.getZ()) <= 0.05);
      assertTrue(Math.abs(startPose.getYaw() - interpolatedOffsetAfterOneSecond.getYaw()) <= 0.05);
      assertTrue(Math.abs(startPose.getPitch() - interpolatedOffsetAfterOneSecond.getPitch()) <= 0.05);
      assertTrue(Math.abs(startPose.getRoll() - interpolatedOffsetAfterOneSecond.getRoll()) <= 0.05);

   }

   @EstimatedDuration(duration = 0.3)
   @Test(timeout = 30000)
   public void testMaxCorrectionSpeedClipWorksWhenTranslationAndRotationOffsetsAreBig()
   {
      int numberOfTicks = 2000;
      alphaFilterBreakFrequency.set(0.6);

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, worldFrame,
            alphaFilterBreakFrequency, dt, true);

      FramePose startPose = new FramePose(worldFrame);
      FramePose goalPose = new FramePose(worldFrame);
      FramePose interpolatedOffset = new FramePose(worldFrame);
      FramePose interpolatedOffsetAfterOneSecond = new FramePose(worldFrame);

      
      //test when Translation is bigger than rotation
      startPose.setToZero(worldFrame);
      Vector3d translation = new Vector3d(1.0, 0.0, 0.0); 
      Quat4d rotation = new Quat4d();
      RotationFunctions.setQuaternionBasedOnYawPitchRoll(rotation, 0.9, 0.0, 0.0);
      goalPose.setPose(translation, rotation);

      clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startPose, goalPose, 1.0);

      for (int j = 0; j < numberOfTicks; j++)
      {
         clippedSpeedOffsetErrorInterpolator.interpolateError(interpolatedOffset);

         if (j == 999)
            interpolatedOffsetAfterOneSecond.setPose(interpolatedOffset);
      }

      assertTrue(Math.abs(startPose.getX() - interpolatedOffsetAfterOneSecond.getX()) <= 0.05);
      assertTrue(Math.abs(startPose.getY() - interpolatedOffsetAfterOneSecond.getY()) <= 0.05);
      assertTrue(Math.abs(startPose.getZ() - interpolatedOffsetAfterOneSecond.getZ()) <= 0.05);
      assertTrue(Math.abs(startPose.getYaw() - interpolatedOffsetAfterOneSecond.getYaw()) <= 0.05);
      assertTrue(Math.abs(startPose.getPitch() - interpolatedOffsetAfterOneSecond.getPitch()) <= 0.05);
      assertTrue(Math.abs(startPose.getRoll() - interpolatedOffsetAfterOneSecond.getRoll()) <= 0.05);
      
      //test when rotation is bigger than translation
      startPose.setToZero(worldFrame);
      translation = new Vector3d(0.9, 0.0, 0.0); 
      rotation = new Quat4d();
      RotationFunctions.setQuaternionBasedOnYawPitchRoll(rotation, 0.0, 1.0, 0.0);
      goalPose.setPose(translation, rotation);
      
      clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startPose, goalPose, 1.0);
      
      for (int j = 0; j < numberOfTicks; j++)
      {
         clippedSpeedOffsetErrorInterpolator.interpolateError(interpolatedOffset);
         
         if (j == 999)
            interpolatedOffsetAfterOneSecond.setPose(interpolatedOffset);
      }
      
      assertTrue(Math.abs(startPose.getX() - interpolatedOffsetAfterOneSecond.getX()) <= 0.05);
      assertTrue(Math.abs(startPose.getY() - interpolatedOffsetAfterOneSecond.getY()) <= 0.05);
      assertTrue(Math.abs(startPose.getZ() - interpolatedOffsetAfterOneSecond.getZ()) <= 0.05);
      assertTrue(Math.abs(startPose.getYaw() - interpolatedOffsetAfterOneSecond.getYaw()) <= 0.05);
      assertTrue(Math.abs(startPose.getPitch() - interpolatedOffsetAfterOneSecond.getPitch()) <= 0.05);
      assertTrue(Math.abs(startPose.getRoll() - interpolatedOffsetAfterOneSecond.getRoll()) <= 0.05);

   }

   @EstimatedDuration(duration = 1.0)
   @Test(timeout = 60000)
   public void testRotationCorrectionIsActuallyDeactivatedWhenAskedTo()
   {
      Random random = new Random();
      int numberOfTicks = 2000;
      alphaFilterBreakFrequency.set(0.6);

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, worldFrame,
            alphaFilterBreakFrequency, dt, false);

      for (int i = 0; i < 1000; i++)
      {
         FramePose startPose = new FramePose(worldFrame);
         FramePose goalPose = new FramePose(worldFrame);
         FramePose interpolatedOffset = new FramePose(worldFrame);

         startPose.setToZero(worldFrame);
         goalPose.setPose(new Vector3d(), RandomTools.generateRandomQuaternion(random, 0.08));

         clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startPose, goalPose, 1.0);

         for (int j = 0; j < numberOfTicks; j++)
         {
            clippedSpeedOffsetErrorInterpolator.interpolateError(interpolatedOffset);
         }
         assertTrue(interpolatedOffset.epsilonEquals(startPose, 1e-8));
      }

   }

}
