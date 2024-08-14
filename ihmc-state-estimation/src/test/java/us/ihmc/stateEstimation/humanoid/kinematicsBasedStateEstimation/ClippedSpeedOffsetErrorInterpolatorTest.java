package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.subscribers.TimeStampedTransformBuffer;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import static org.junit.jupiter.api.Assertions.*;

public class ClippedSpeedOffsetErrorInterpolatorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private RigidBodyTransform referenceFrameToBeCorrectedTransform = new RigidBodyTransform();
   private final Vector3D referenceFrameToBeCorrectedTransform_Translation = new Vector3D();
   private final Quaternion referenceFrameToBeCorrectedTransform_Rotation = new Quaternion(0.0, 0.0, 0.0, 1.0);

   int numberOfWaypoints = 2;
   TimeStampedTransformBuffer referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame = new TimeStampedTransformBuffer(numberOfWaypoints);

   private final ReferenceFrame referenceFrameToBeCorrected = new ReferenceFrame("referenceFrameToBeCorrected", worldFrame)
   {

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.set(referenceFrameToBeCorrectedTransform);

      }
   };
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double dt = 0.001;

   @BeforeEach
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void showMemoryAfterTests()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testClippedSpeedOffsetErrorInterpolator()
   {
      boolean visualize = false;

      double dt = 0.001;
      YoRegistry registry = new YoRegistry("Test");
      RigidBodyTransform referenceFrameToBeCorrectedTransform = new RigidBodyTransform();

      ReferenceFrame referenceFrameToBeCorrected = new ReferenceFrame("referenceFrameToBeCorrected", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(referenceFrameToBeCorrectedTransform);
         }
      };

      ClippedSpeedOffsetErrorInterpolatorParameters interpolatorParameters = new ClippedSpeedOffsetErrorInterpolatorParameters();
      interpolatorParameters.setIsRotationCorrectionEnabled(true);
      interpolatorParameters.setDeadZoneSizes(0.0, 0.0, 0.0, 0.0);
      interpolatorParameters.setMaximumTranslationAndRotationSpeed(10000.0, 10000.0);
      interpolatorParameters.setBreakFrequency(100000.0);
      interpolatorParameters.setIsRotationCorrectionEnabled(true);
      
      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry,
                                                                                                                        referenceFrameToBeCorrected,
                                                                                                                        dt,
                                                                                                                        interpolatorParameters);

      // No dead zone, no speed limits, and no filter: Should move to full range in one tick


      FramePose3D startOffsetError = new FramePose3D(worldFrame);
      FramePose3D goalOffsetError = new FramePose3D(worldFrame);

      // Simple test with no error:
      double alphaFilterPosition = 1.0;
      clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startOffsetError, goalOffsetError, alphaFilterPosition);

      FramePose3D offsetPoseToPack = new FramePose3D(worldFrame);
      clippedSpeedOffsetErrorInterpolator.interpolateError(offsetPoseToPack);
      EuclidFrameTestTools.assertEquals(offsetPoseToPack, new FramePose3D(worldFrame), 1e-7);
      clippedSpeedOffsetErrorInterpolator.interpolateError(offsetPoseToPack);
      EuclidFrameTestTools.assertEquals(offsetPoseToPack, new FramePose3D(worldFrame), 1e-7);

      // Simple test with some error:

      startOffsetError.getPosition().set(new Point3D(0.01, 0.02, 0.03));
      startOffsetError.getOrientation().setYawPitchRoll(-0.023, 0.179, 0.11);
      goalOffsetError.getPosition().set(new Point3D(0.02, 0.037, -0.04));
      goalOffsetError.getOrientation().setYawPitchRoll(0.17, 0.114, -0.005);

      clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startOffsetError, goalOffsetError, alphaFilterPosition);
      clippedSpeedOffsetErrorInterpolator.interpolateError(offsetPoseToPack);

      FramePose3D expectedPose = new FramePose3D(worldFrame);
      expectedPose.set(startOffsetError);
      expectedPose.getOrientation().setYawPitchRoll(expectedPose.getYaw(), 0.0, 0.0);

      EuclidFrameTestTools.assertEquals(offsetPoseToPack, expectedPose, 1e-7);
      clippedSpeedOffsetErrorInterpolator.interpolateError(offsetPoseToPack);
      expectedPose.set(goalOffsetError);
      expectedPose.getOrientation().setYawPitchRoll(expectedPose.getYaw(), 0.0, 0.0);
      EuclidFrameTestTools.assertEquals(offsetPoseToPack, expectedPose, 1e-7);

      SimulationConstructionSet scs = null;
      if (visualize)
      {
         SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
         Robot robot = new Robot(getClass().getSimpleName());
         scs = new SimulationConstructionSet(robot, parameters);
         scs.addYoRegistry(registry);
         scs.startOnAThread();
      }
      
      // With some speed limits, should follow those speed limits:

      startOffsetError.getPosition().set(new Point3D(0.01, 0.001, 0.0));
      startOffsetError.getOrientation().setYawPitchRoll(0.1, 0.2, 0.3);
      goalOffsetError.getPosition().set(new Point3D(0.02, 0.002, 0.0));
      goalOffsetError.getOrientation().setYawPitchRoll(0.1, 0.2, 0.3);
      
      double distanceFromStartToGoal = startOffsetError.getPosition().distance(goalOffsetError.getPosition());
      clippedSpeedOffsetErrorInterpolator.setMaximumTranslationAndRotationVelocity(distanceFromStartToGoal, 1.0);

      clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startOffsetError, goalOffsetError, alphaFilterPosition);

      Vector3D translationPerTick = new Vector3D();
      translationPerTick.sub(goalOffsetError.getPosition(), startOffsetError.getPosition());
      translationPerTick.scale(dt);

      int numberOfTicks = (int) Math.round(1.0 / dt);
      Point3D expectedPosition = new Point3D(startOffsetError.getPosition());

      for (int i = 0; i < numberOfTicks; i++)
      {
         clippedSpeedOffsetErrorInterpolator.interpolateError(offsetPoseToPack);
         assertTrue(expectedPosition.epsilonEquals(offsetPoseToPack.getPosition(), 1e-7));
         expectedPosition.add(translationPerTick);

         if (visualize)
         {
            scs.tickAndUpdate();
            scs.setTime(scs.getTime() + 1.0);
         }
      }
      
      // With rotation speed limits:

      double startYaw = 0.01;
      double endYaw = 0.02;
      
      startOffsetError.getPosition().set(new Point3D(0.01, 0.02, 0.03));
      startOffsetError.getOrientation().setYawPitchRoll(startYaw, 0.02, 0.03);
      goalOffsetError.getPosition().set(new Point3D(0.02, -0.01, 0.0378));
      goalOffsetError.getOrientation().setYawPitchRoll(endYaw, 0.014, -0.045);

      double rotationFromStartToGoal = endYaw - startYaw;

      clippedSpeedOffsetErrorInterpolator.setIsRotationCorrectionEnabled(true);
      clippedSpeedOffsetErrorInterpolator.setMaximumTranslationAndRotationVelocity(1.0, rotationFromStartToGoal);
      clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startOffsetError, goalOffsetError, alphaFilterPosition);

      double rotationPerTick = rotationFromStartToGoal * dt;

      numberOfTicks = (int) Math.round(1.0 / dt);
      double expectedYaw = startYaw;

      for (int i = 0; i < numberOfTicks; i++)
      {
         clippedSpeedOffsetErrorInterpolator.interpolateError(offsetPoseToPack);

         org.junit.jupiter.api.Assertions.assertEquals(expectedYaw, offsetPoseToPack.getYaw(), 1e-7);
         expectedYaw += rotationPerTick;

         if (visualize)
         {
            scs.setTime(scs.getTime() + 1.0);
            scs.tickAndUpdate();
         }
      }

      if (visualize)
      {
         ThreadTools.sleepForever();
      }

   }

   @Test
   public void testMaxTranslationalCorrectionSpeedClip()
   {
      Random random = new Random(8765L);

      boolean visualize = false;

      YoRegistry registry = new YoRegistry("Test");

      int numberOfTicks = 10000;
      double dt = 0.001;

      ClippedSpeedOffsetErrorInterpolatorParameters interpolationParameters = new ClippedSpeedOffsetErrorInterpolatorParameters();
      interpolationParameters.setIsRotationCorrectionEnabled(false);
      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry,
                                                                                                                        referenceFrameToBeCorrected,
                                                                                                                        dt,
                                                                                                                        interpolationParameters);

      SimulationConstructionSet scs = null;
      if (visualize)
      {
         SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
         parameters.setDataBufferSize(12000);
         Robot robot = new Robot(getClass().getSimpleName());
         scs = new SimulationConstructionSet(robot, parameters);
         scs.addYoRegistry(registry);
         scs.startOnAThread();
      }

      generateRandomReferenceFrameToBeCorrectedWaypoints(random, 0, numberOfTicks);
      TimeStampedTransform3D temporaryTimeStampedTransform = new TimeStampedTransform3D();

      referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.findTransform(0, temporaryTimeStampedTransform);
      referenceFrameToBeCorrectedTransform = temporaryTimeStampedTransform.getTransform3D();
      referenceFrameToBeCorrectedTransform_Translation.set(referenceFrameToBeCorrectedTransform.getTranslation());
      referenceFrameToBeCorrectedTransform_Rotation.set(referenceFrameToBeCorrectedTransform.getRotation());
      referenceFrameToBeCorrected.update();

      RigidBodyTransform startPoseTransform = new RigidBodyTransform();
      Vector3D startPose_Translation = new Vector3D(0.0, 0.0, 0.0);
      Quaternion startPose_Rotation = new Quaternion(0.0, 0.0, 0.0, 1.0);

      startPoseTransform.setIdentity();
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), referenceFrameToBeCorrectedTransform_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), startPose_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
      startPoseTransform.multiply(new RigidBodyTransform(startPose_Rotation, new Vector3D()));

      FramePose3D startPose = new FramePose3D(worldFrame);
      startPose.set(startPoseTransform);

      RigidBodyTransform goalPoseTransform = new RigidBodyTransform();
      Vector3D goalPose_Translation = new Vector3D(1.0, 1.0, 1.0);
      Quaternion goalPose_Rotation = new Quaternion(0.011, 0.021, 0.031, 1.0);

      goalPoseTransform.setIdentity();
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), referenceFrameToBeCorrectedTransform_Translation));
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), goalPose_Translation));
      goalPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
      goalPoseTransform.multiply(new RigidBodyTransform(goalPose_Rotation, new Vector3D()));

      FramePose3D goalPose = new FramePose3D(worldFrame);
      goalPose.set(goalPoseTransform);

      FramePose3D interpolatedPose = new FramePose3D(startPose);
      FramePose3D interpolatedPoseOneSecondEarlier = new FramePose3D(startPose);

      clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startPose, goalPose, 1.0);

      for (int j = 0; j < numberOfTicks; j++)
      {
         referenceFrameToBeCorrected.update();
         clippedSpeedOffsetErrorInterpolator.interpolateError(interpolatedPose);

         if (j > 0 && (j % 999) == 0)
         {
            FramePoint3D interpolatedPoseFramePointToPack = new FramePoint3D();
            FrameQuaternion interpolatedPoseOrientationToPack = new FrameQuaternion();
            interpolatedPose.get(interpolatedPoseFramePointToPack, interpolatedPoseOrientationToPack);

            FramePoint3D interpolatedPoseOneSecondEarlierFramePointToPack = new FramePoint3D();
            FrameQuaternion interpolatedPoseOneSecondEarlierOrientationToPack = new FrameQuaternion();
            interpolatedPoseOneSecondEarlier.get(interpolatedPoseOneSecondEarlierFramePointToPack, interpolatedPoseOneSecondEarlierOrientationToPack);

            Vector3D translationDisplacement = new Vector3D();
            translationDisplacement.sub(interpolatedPoseOneSecondEarlierFramePointToPack, interpolatedPoseFramePointToPack);

            FrameQuaternion rotationDisplacement = new FrameQuaternion();
            rotationDisplacement.difference(interpolatedPoseOrientationToPack, interpolatedPoseOneSecondEarlierOrientationToPack);
            AxisAngle rotationDisplacementAngle = new AxisAngle(rotationDisplacement);

            assertTrue(Math.abs(translationDisplacement.length()) >= 0.01);
            assertTrue(Math.abs(translationDisplacement.length()) <= 0.05);
            assertTrue(Math.abs(rotationDisplacementAngle.getAngle()) <= 1e-7);

            interpolatedPoseOneSecondEarlier.set(interpolatedPose);
         }
         
         if (visualize)
         {
            scs.setTime(scs.getTime() + 1.0);
            scs.tickAndUpdate();
         }
      }

      if (visualize)
      {
         ThreadTools.sleepForever();
      }
   }

   @Test
   public void testMaxRotationalCorrectionSpeedClip()
   {
      int numberOfTicks = 10000;
      Random random = new Random(999L);

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry,
                                                                                                                        referenceFrameToBeCorrected,
                                                                                                                        dt);

      generateRandomReferenceFrameToBeCorrectedWaypoints(random, 0, numberOfTicks);
      TimeStampedTransform3D temporaryTimeStampedTransform = new TimeStampedTransform3D();

      referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.findTransform(0, temporaryTimeStampedTransform);
      referenceFrameToBeCorrectedTransform = temporaryTimeStampedTransform.getTransform3D();
      referenceFrameToBeCorrectedTransform_Translation.set(referenceFrameToBeCorrectedTransform.getTranslation());
      referenceFrameToBeCorrectedTransform_Rotation.set(referenceFrameToBeCorrectedTransform.getRotation());
      referenceFrameToBeCorrected.update();

      RigidBodyTransform startPoseTransform = new RigidBodyTransform();
      Vector3D startPose_Translation = new Vector3D(0.0, 0.0, 0.0);
      Quaternion startPose_Rotation = new Quaternion(0.0, 0.0, 0.0, 1.0);

      startPoseTransform.setIdentity();
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), referenceFrameToBeCorrectedTransform_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), startPose_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
      startPoseTransform.multiply(new RigidBodyTransform(startPose_Rotation, new Vector3D()));

      FramePose3D startPose = new FramePose3D(worldFrame);
      startPose.set(startPoseTransform);

      RigidBodyTransform goalPoseTransform = new RigidBodyTransform();
      Vector3D goalPose_Translation = new Vector3D(0.0, 0.0, 0.0);
      Quaternion goalPose_Rotation = new Quaternion();
      goalPose_Rotation.setYawPitchRoll(1.0, 0.3, -0.11);

      goalPoseTransform.setIdentity();
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), referenceFrameToBeCorrectedTransform_Translation));
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), goalPose_Translation));
      goalPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
      goalPoseTransform.multiply(new RigidBodyTransform(goalPose_Rotation, new Vector3D()));

      FramePose3D goalPose = new FramePose3D(worldFrame);
      goalPose.set(goalPoseTransform);

      FramePose3D interpolatedPose = new FramePose3D(startPose);
      FramePose3D interpolatedPoseOneSecondEarlier = new FramePose3D(startPose);

      clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startPose, goalPose, 1.0);

      for (int j = 0; j < numberOfTicks; j++)
      {
         referenceFrameToBeCorrected.update();
         clippedSpeedOffsetErrorInterpolator.interpolateError(interpolatedPose);

         if (j > 0 && (j % 999) == 0)
         {
            FramePoint3D interpolatedPoseFramePointToPack = new FramePoint3D();
            FrameQuaternion interpolatedPoseOrientationToPack = new FrameQuaternion();
            interpolatedPose.get(interpolatedPoseFramePointToPack, interpolatedPoseOrientationToPack);

            FramePoint3D interpolatedPoseOneSecondEarlierFramePointToPack = new FramePoint3D();
            FrameQuaternion interpolatedPoseOneSecondEarlierOrientationToPack = new FrameQuaternion();
            interpolatedPoseOneSecondEarlier.get(interpolatedPoseOneSecondEarlierFramePointToPack, interpolatedPoseOneSecondEarlierOrientationToPack);

            Vector3D translationDisplacement = new Vector3D(interpolatedPoseOneSecondEarlierFramePointToPack.getX() - interpolatedPoseFramePointToPack.getX(),
                                                            interpolatedPoseOneSecondEarlierFramePointToPack.getY() - interpolatedPoseFramePointToPack.getY(),
                                                            interpolatedPoseOneSecondEarlierFramePointToPack.getZ() - interpolatedPoseFramePointToPack.getZ());

            FrameQuaternion rotationDisplacement = new FrameQuaternion();
            rotationDisplacement.difference(interpolatedPoseOrientationToPack, interpolatedPoseOneSecondEarlierOrientationToPack);
            AxisAngle rotationDisplacementAngle = new AxisAngle(rotationDisplacement);

            assertTrue(Math.abs(translationDisplacement.length()) <= 0.05);
            assertTrue(Math.abs(rotationDisplacementAngle.getAngle()) >= 0.01);
            assertTrue(Math.abs(rotationDisplacementAngle.getAngle()) <= 0.05);

            interpolatedPoseOneSecondEarlier.set(interpolatedPose);
         }
      }
   }


   @Test
   public void testMaxCorrectionSpeedClipWorksWhenTranslationAndRotationOffsetsAreBig()
   {
      int numberOfTicks = 10000;
      Random random = new Random(1234L);

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry,
                                                                                                                        referenceFrameToBeCorrected,
                                                                                                                        dt);

      generateRandomReferenceFrameToBeCorrectedWaypoints(random, 0, numberOfTicks);
      TimeStampedTransform3D temporaryTimeStampedTransform = new TimeStampedTransform3D();

      referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.findTransform(0, temporaryTimeStampedTransform);
      referenceFrameToBeCorrectedTransform = temporaryTimeStampedTransform.getTransform3D();
      referenceFrameToBeCorrectedTransform_Translation.set(referenceFrameToBeCorrectedTransform.getTranslation());
      referenceFrameToBeCorrectedTransform_Rotation.set(referenceFrameToBeCorrectedTransform.getRotation());
      referenceFrameToBeCorrected.update();

      //test when rotation is bigger than translation
      RigidBodyTransform startPoseTransform = new RigidBodyTransform();
      Vector3D startPose_Translation = new Vector3D(0.0, 0.0, 0.0);
      Quaternion startPose_Rotation = new Quaternion(0.0, 0.0, 0.0, 1.0);

      startPoseTransform.setIdentity();
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), referenceFrameToBeCorrectedTransform_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), startPose_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
      startPoseTransform.multiply(new RigidBodyTransform(startPose_Rotation, new Vector3D()));

      FramePose3D startPose = new FramePose3D(worldFrame);
      startPose.set(startPoseTransform);

      RigidBodyTransform goalPoseTransform = new RigidBodyTransform();
      Vector3D goalPose_Translation = new Vector3D(0.2, 0.2, 0.2);
      Quaternion goalPose_Rotation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      goalPose_Rotation.setYawPitchRoll(1.0, 1.0, 1.0);

      goalPoseTransform.setIdentity();
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), referenceFrameToBeCorrectedTransform_Translation));
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), goalPose_Translation));
      goalPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
      goalPoseTransform.multiply(new RigidBodyTransform(goalPose_Rotation, new Vector3D()));

      FramePose3D goalPose = new FramePose3D(worldFrame);
      goalPose.set(goalPoseTransform);

      FramePose3D interpolatedPose = new FramePose3D(startPose);
      FramePose3D interpolatedPoseOneSecondEarlier = new FramePose3D(startPose);

      clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startPose, goalPose, 1.0);

      for (int j = 0; j < numberOfTicks; j++)
      {
         referenceFrameToBeCorrected.update();
         clippedSpeedOffsetErrorInterpolator.interpolateError(interpolatedPose);

         if (j > 0 && (j % 999) == 0)
         {
            FramePoint3D interpolatedPoseFramePointToPack = new FramePoint3D();
            FrameQuaternion interpolatedPoseOrientationToPack = new FrameQuaternion();
            interpolatedPose.get(interpolatedPoseFramePointToPack, interpolatedPoseOrientationToPack);

            FramePoint3D interpolatedPoseOneSecondEarlierFramePointToPack = new FramePoint3D();
            FrameQuaternion interpolatedPoseOneSecondEarlierOrientationToPack = new FrameQuaternion();
            interpolatedPoseOneSecondEarlier.get(interpolatedPoseOneSecondEarlierFramePointToPack, interpolatedPoseOneSecondEarlierOrientationToPack);

            Vector3D translationDisplacement = new Vector3D(interpolatedPoseOneSecondEarlierFramePointToPack.getX() - interpolatedPoseFramePointToPack.getX(),
                                                            interpolatedPoseOneSecondEarlierFramePointToPack.getY() - interpolatedPoseFramePointToPack.getY(),
                                                            interpolatedPoseOneSecondEarlierFramePointToPack.getZ() - interpolatedPoseFramePointToPack.getZ());

            FrameQuaternion rotationDisplacement = new FrameQuaternion();
            rotationDisplacement.difference(interpolatedPoseOrientationToPack, interpolatedPoseOneSecondEarlierOrientationToPack);
            AxisAngle rotationDisplacementAngle = new AxisAngle(rotationDisplacement);

            assertTrue(Math.abs(translationDisplacement.length()) <= 0.05);
            assertTrue(Math.abs(rotationDisplacementAngle.getAngle()) <= 0.05, "rotationDisplacementAngle.getAngle() = " + rotationDisplacementAngle.getAngle());

            interpolatedPoseOneSecondEarlier.set(interpolatedPose);
         }
      }

      //test when translation is bigger than rotation
      startPoseTransform = new RigidBodyTransform();
      startPose_Translation = new Vector3D(0.0, 0.0, 0.0);
      startPose_Rotation = new Quaternion(0.0, 0.0, 0.0, 1.0);

      startPoseTransform.setIdentity();
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), referenceFrameToBeCorrectedTransform_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), startPose_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
      startPoseTransform.multiply(new RigidBodyTransform(startPose_Rotation, new Vector3D()));

      startPose = new FramePose3D(worldFrame);
      startPose.set(startPoseTransform);

      goalPoseTransform = new RigidBodyTransform();
      goalPose_Translation = new Vector3D(1.0, 1.0, 1.0);
      goalPose_Rotation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      goalPose_Rotation.setYawPitchRoll(0.2, 0.2, 0.2);

      goalPoseTransform.setIdentity();
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), referenceFrameToBeCorrectedTransform_Translation));
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), goalPose_Translation));
      goalPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
      goalPoseTransform.multiply(new RigidBodyTransform(goalPose_Rotation, new Vector3D()));

      goalPose = new FramePose3D(worldFrame);
      goalPose.set(goalPoseTransform);

      interpolatedPose = new FramePose3D(startPose);
      interpolatedPoseOneSecondEarlier = new FramePose3D(startPose);

      clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startPose, goalPose, 1.0);

      for (int j = 0; j < numberOfTicks; j++)
      {
         referenceFrameToBeCorrected.update();
         clippedSpeedOffsetErrorInterpolator.interpolateError(interpolatedPose);

         if (j > 0 && (j % 999) == 0)
         {
            FramePoint3D interpolatedPoseFramePointToPack = new FramePoint3D();
            FrameQuaternion interpolatedPoseOrientationToPack = new FrameQuaternion();
            interpolatedPose.get(interpolatedPoseFramePointToPack, interpolatedPoseOrientationToPack);

            FramePoint3D interpolatedPoseOneSecondEarlierFramePointToPack = new FramePoint3D();
            FrameQuaternion interpolatedPoseOneSecondEarlierOrientationToPack = new FrameQuaternion();
            interpolatedPoseOneSecondEarlier.get(interpolatedPoseOneSecondEarlierFramePointToPack, interpolatedPoseOneSecondEarlierOrientationToPack);

            Vector3D translationDisplacement = new Vector3D(interpolatedPoseOneSecondEarlierFramePointToPack.getX() - interpolatedPoseFramePointToPack.getX(),
                                                            interpolatedPoseOneSecondEarlierFramePointToPack.getY() - interpolatedPoseFramePointToPack.getY(),
                                                            interpolatedPoseOneSecondEarlierFramePointToPack.getZ() - interpolatedPoseFramePointToPack.getZ());

            FrameQuaternion rotationDisplacement = new FrameQuaternion();
            rotationDisplacement.difference(interpolatedPoseOrientationToPack, interpolatedPoseOneSecondEarlierOrientationToPack);
            AxisAngle rotationDisplacementAngle = new AxisAngle(rotationDisplacement);

            assertTrue(Math.abs(translationDisplacement.length()) <= 0.05);
            assertTrue(Math.abs(rotationDisplacementAngle.getAngle()) <= 0.05);

            interpolatedPoseOneSecondEarlier.set(interpolatedPose);
         }
      }
   }

   @Test
   public void testRotationCorrectionIsActuallyDeactivatedWhenAskedTo()
   {
      Random random = new Random(1984L);
      int numberOfTicks = 2000;

      ClippedSpeedOffsetErrorInterpolatorParameters parameters = new ClippedSpeedOffsetErrorInterpolatorParameters();
      parameters.setIsRotationCorrectionEnabled(false);
      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry,
                                                                                                                        referenceFrameToBeCorrected,
                                                                                                                        dt,
                                                                                                                        parameters);

      generateRandomReferenceFrameToBeCorrectedWaypoints(random, 0, numberOfTicks);
      TimeStampedTransform3D temporaryTimeStampedTransform = new TimeStampedTransform3D();

      for (int i = 0; i < 20; i++)
      {
         referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.findTransform(0, temporaryTimeStampedTransform);
         referenceFrameToBeCorrectedTransform = temporaryTimeStampedTransform.getTransform3D();
         referenceFrameToBeCorrectedTransform_Translation.set(referenceFrameToBeCorrectedTransform.getTranslation());
         referenceFrameToBeCorrectedTransform_Rotation.set(referenceFrameToBeCorrectedTransform.getRotation());
         referenceFrameToBeCorrected.update();

         RigidBodyTransform startPoseTransform = new RigidBodyTransform();
         Vector3D startPose_Translation = new Vector3D(0.0, 0.0, 0.0);
         Quaternion startPose_Rotation = new Quaternion(0.0, 0.0, 0.0, 1.0);

         startPoseTransform.setIdentity();
         startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), referenceFrameToBeCorrectedTransform_Translation));
         startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), startPose_Translation));
         startPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
         startPoseTransform.multiply(new RigidBodyTransform(startPose_Rotation, new Vector3D()));

         FramePose3D startPose = new FramePose3D(worldFrame);
         startPose.set(startPoseTransform);

         RigidBodyTransform goalPoseTransform = new RigidBodyTransform();
         Vector3D goalPose_Translation = new Vector3D(0.0, 0.0, 0.0);
         Quaternion goalPose_Rotation = EuclidCoreRandomTools.nextQuaternion(random, 0.04);

         goalPoseTransform.setIdentity();
         goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), referenceFrameToBeCorrectedTransform_Translation));
         goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), goalPose_Translation));
         goalPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
         goalPoseTransform.multiply(new RigidBodyTransform(goalPose_Rotation, new Vector3D()));

         FramePose3D goalPose = new FramePose3D(worldFrame);
         goalPose.set(goalPoseTransform);

         FramePose3D interpolatedPose = new FramePose3D(startPose);

         clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startPose, goalPose, 1.0);

         for (int timestamp = 0; timestamp < numberOfTicks; timestamp++)
         {
            referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.findTransform(timestamp, temporaryTimeStampedTransform);
            referenceFrameToBeCorrectedTransform.set(temporaryTimeStampedTransform.getTransform3D());
            referenceFrameToBeCorrected.update();

            clippedSpeedOffsetErrorInterpolator.interpolateError(interpolatedPose);
         }

         ////////////////update startPose for asserts////
         referenceFrameToBeCorrectedTransform_Translation.set(referenceFrameToBeCorrectedTransform.getTranslation());
         referenceFrameToBeCorrectedTransform_Rotation.set(referenceFrameToBeCorrectedTransform.getRotation());

         startPoseTransform.setIdentity();
         startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), referenceFrameToBeCorrectedTransform_Translation));
         startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), startPose_Translation));
         startPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
         startPoseTransform.multiply(new RigidBodyTransform(startPose_Rotation, new Vector3D()));
         startPose.set(startPoseTransform);

         EuclidFrameTestTools.assertEquals(interpolatedPose, startPose, 1e-4);
      }
   }

   @Test
   public void testErrorRotationCheckIsBehavingProperly()
   {
      Random random = new Random(1776L);

      CorrectedPelvisPoseErrorTooBigChecker correctedPelvisPoseErrorTooBigChecker = new CorrectedPelvisPoseErrorTooBigChecker(registry);

      FramePose3D startPose = new FramePose3D(worldFrame, new Point3D(), new Quaternion(0.0, 0.0, 0.0, 1.0));
      FramePose3D goalPose = new FramePose3D(worldFrame);
      Quaternion goalOrientation = new Quaternion();

      for (int i = 0; i < 200; i++)
      {
         goalPose.set(EuclidCoreRandomTools.nextPoint3D(random, 0.1, 0.1, 0.1), EuclidCoreRandomTools.nextQuaternion(random, Math.toRadians(9.2)));
         assertFalse(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));
      }

      goalOrientation.setYawPitchRoll(Math.toRadians(10.1), Math.toRadians(0.0), Math.toRadians(0.0));
      goalPose.set(EuclidCoreRandomTools.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(10.1), Math.toRadians(0.0));
      goalPose.set(EuclidCoreRandomTools.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(10.1));
      goalPose.set(EuclidCoreRandomTools.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(10.2), Math.toRadians(10.3), Math.toRadians(0.0));
      goalPose.set(EuclidCoreRandomTools.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(10.2), Math.toRadians(0.0), Math.toRadians(10.1));
      goalPose.set(EuclidCoreRandomTools.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(10.2), Math.toRadians(10.1));
      goalPose.set(EuclidCoreRandomTools.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(20.0), Math.toRadians(10.2), Math.toRadians(10.1));
      goalPose.set(EuclidCoreRandomTools.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(-10.1), Math.toRadians(0.0), Math.toRadians(0.0));
      goalPose.set(EuclidCoreRandomTools.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(-10.1), Math.toRadians(0.0));
      goalPose.set(EuclidCoreRandomTools.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(-10.1));
      goalPose.set(EuclidCoreRandomTools.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(-10.2), Math.toRadians(-10.3), Math.toRadians(0.0));
      goalPose.set(EuclidCoreRandomTools.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(-10.2), Math.toRadians(0.0), Math.toRadians(-10.1));
      goalPose.set(EuclidCoreRandomTools.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(-10.2), Math.toRadians(-10.1));
      goalPose.set(EuclidCoreRandomTools.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(-20.0), Math.toRadians(-10.2), Math.toRadians(-10.1));
      goalPose.set(EuclidCoreRandomTools.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

   }

   private void generateRandomReferenceFrameToBeCorrectedWaypoints(Random random, long firstTimestamp, long lastTimestamp)
   {
      Vector3D translation = new Vector3D();
      Quaternion rotation = new Quaternion(0.0, 0.0, 0.0, 1.0);

      translation = EuclidCoreRandomTools.nextVector3D(random);
      rotation = EuclidCoreRandomTools.nextQuaternion(random);
      putReferenceFrameToBeCorrectedWaypointInTransformBuffer(firstTimestamp, translation, rotation);

      translation = EuclidCoreRandomTools.nextVector3D(random);
      rotation = EuclidCoreRandomTools.nextQuaternion(random);
      putReferenceFrameToBeCorrectedWaypointInTransformBuffer(lastTimestamp, translation, rotation);
   }

   private void putReferenceFrameToBeCorrectedWaypointInTransformBuffer(long timeStamp, Vector3D translation, Quaternion rotation)
   {
      RigidBodyTransform temporaryReferenceFrameToBeCorrectedTransformInWorldFrame = new RigidBodyTransform();
      temporaryReferenceFrameToBeCorrectedTransformInWorldFrame.getTranslation().set(translation);
      temporaryReferenceFrameToBeCorrectedTransformInWorldFrame.getRotation().set(rotation);
      referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.put(temporaryReferenceFrameToBeCorrectedTransformInWorldFrame, timeStamp);
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(ClippedSpeedOffsetErrorInterpolator.class, ClippedSpeedOffsetErrorInterpolatorTest.class);
   }
}
