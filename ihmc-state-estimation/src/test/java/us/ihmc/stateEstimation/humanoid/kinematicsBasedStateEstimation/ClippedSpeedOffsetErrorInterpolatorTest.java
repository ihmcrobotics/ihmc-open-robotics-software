package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.subscribers.TimeStampedTransformBuffer;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ClippedSpeedOffsetErrorInterpolatorTest
{
   SimulationTestingParameters simulationTestingParameters = new SimulationTestingParameters();

   SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
   SimulationConstructionSet scs;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private RigidBodyTransform referenceFrameToBeCorrectedTransform = new RigidBodyTransform();
   private final Vector3D referenceFrameToBeCorrectedTransform_Translation = new Vector3D();
   private final Quaternion referenceFrameToBeCorrectedTransform_Rotation = new Quaternion(0.0,0.0,0.0,1.0);

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
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble alphaFilterBreakFrequency = new YoDouble("alphaFilterBreakFrequency", registry);
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
   public void testSimpleCases()
   {
      boolean visualize = true;
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      SimulationConstructionSet scs = null;

      boolean correctRotation = false;
      double dt = 0.001;
      YoVariableRegistry registry = new YoVariableRegistry("Test");
      YoDouble alphaFilterBreakFrequency = new YoDouble("alphaFilterBreakFrequency", registry);
      alphaFilterBreakFrequency.set(100.0);
      RigidBodyTransform referenceFrameToBeCorrectedTransform = new RigidBodyTransform();

      if (visualize)
      {
         Robot robot = new Robot(getClass().getSimpleName());
         scs  = new SimulationConstructionSet(robot, parameters);
         scs.addYoVariableRegistry(registry);
         scs.startOnAThread();
      }

      
      ReferenceFrame referenceFrameToBeCorrected = new ReferenceFrame("referenceFrameToBeCorrected", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(referenceFrameToBeCorrectedTransform);
         }
      };

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, referenceFrameToBeCorrected,
                                                                                                                        alphaFilterBreakFrequency, dt, correctRotation);


      clippedSpeedOffsetErrorInterpolator.setDeadZoneSizes(0.0, 0.0, 0.0, 0.0);
      clippedSpeedOffsetErrorInterpolator.setMaximumTranslationAndRotationVelocity(10000.0, 10000.0);

      FramePose3D startOffsetError = new FramePose3D(worldFrame);
      FramePose3D goalOffsetError = new FramePose3D(worldFrame);

      double alphaFilterPosition = 1.0;
      clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startOffsetError, goalOffsetError, alphaFilterPosition);

      FramePose3D offsetPoseToPack = new FramePose3D(worldFrame);
      clippedSpeedOffsetErrorInterpolator.interpolateError(offsetPoseToPack);

      assertTrue(offsetPoseToPack.epsilonEquals(new FramePose3D(worldFrame), 1e-7));
      
      
      startOffsetError.setPosition(new Point3D(0.01, 0.02, 0.03));
      goalOffsetError.setPosition(new Point3D(0.02, 0.03, -0.04));
      double initialDistance = startOffsetError.getPositionDistance(goalOffsetError);
      
      clippedSpeedOffsetErrorInterpolator.setInterpolatorInputs(startOffsetError, goalOffsetError, alphaFilterPosition);
      
      int numberOfTicks = 100;
      
      for (int i=0; i<numberOfTicks; i++)
      {
         if (visualize)
         {
            scs.tickAndUpdate();
         }
      clippedSpeedOffsetErrorInterpolator.interpolateError(offsetPoseToPack);
      
      System.out.println(offsetPoseToPack);
      }
      
      if (visualize)
      {
         ThreadTools.sleepForever();
      }

   }
   
   
   @Disabled
   @Test
   public void testRandomTranslationErrorInterpolation()
   {
      Random random = new Random();
      int numberOfTicks = 2000;
      alphaFilterBreakFrequency.set(0.6);

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, referenceFrameToBeCorrected,
            alphaFilterBreakFrequency, dt, false);

      generateRandomReferenceFrameToBeCorrectedWaypoints(0, numberOfTicks);
      TimeStampedTransform3D temporaryTimeStampedTransform = new TimeStampedTransform3D();

      for (int i = 0; i < 1000; i++)
      {
         referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.findTransform(0, temporaryTimeStampedTransform);
         referenceFrameToBeCorrectedTransform = temporaryTimeStampedTransform.getTransform3D();
         referenceFrameToBeCorrectedTransform.getTranslation(referenceFrameToBeCorrectedTransform_Translation);
         referenceFrameToBeCorrectedTransform.getRotation(referenceFrameToBeCorrectedTransform_Rotation);
         referenceFrameToBeCorrected.update();

         RigidBodyTransform startPoseTransform = new RigidBodyTransform();
         Vector3D startPose_Translation = new Vector3D(0.0, 0.0, 0.0);
         Quaternion startPose_Rotation = new Quaternion(0.0,0.0,0.0,1.0);

         startPoseTransform.setIdentity();
         startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
         startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), startPose_Translation));
         startPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
         startPoseTransform.multiply(new RigidBodyTransform(startPose_Rotation, new Vector3D()));

         FramePose3D startPose = new FramePose3D(worldFrame);
         startPose.set(startPoseTransform);

         RigidBodyTransform goalPoseTransform = new RigidBodyTransform();
         Vector3D goalPose_Translation = RandomGeometry.nextVector3D(random, 0.04);
         Quaternion goalPose_Rotation = new Quaternion(0.0,0.0,0.0,1.0);

         goalPoseTransform.setIdentity();
         goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
         goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), goalPose_Translation));
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

         ////////////////update goalPose for asserts////
         referenceFrameToBeCorrectedTransform.getTranslation(referenceFrameToBeCorrectedTransform_Translation);
         referenceFrameToBeCorrectedTransform.getRotation(referenceFrameToBeCorrectedTransform_Rotation);

         goalPoseTransform.setIdentity();
         goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
         goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), goalPose_Translation));
         goalPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
         goalPoseTransform.multiply(new RigidBodyTransform(goalPose_Rotation, new Vector3D()));
         goalPose.set(goalPoseTransform);
         assertTrue(interpolatedPose.epsilonEquals(goalPose, 1e-4));
      }
   }

   //TODO
   @Disabled
   @Test
   public void testRandomRotationErrorInterpolation()
   {
      Random random = new Random();
      int numberOfTicks = 2000;
      alphaFilterBreakFrequency.set(0.6);

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, referenceFrameToBeCorrected,
            alphaFilterBreakFrequency, dt, true);

      generateRandomReferenceFrameToBeCorrectedWaypoints(0, numberOfTicks);
      TimeStampedTransform3D temporaryTimeStampedTransform = new TimeStampedTransform3D();

      for (int i = 0; i < 1000; i++)
      {
         referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.findTransform(0, temporaryTimeStampedTransform);
         referenceFrameToBeCorrectedTransform = temporaryTimeStampedTransform.getTransform3D();
         referenceFrameToBeCorrectedTransform.getTranslation(referenceFrameToBeCorrectedTransform_Translation);
         referenceFrameToBeCorrectedTransform.getRotation(referenceFrameToBeCorrectedTransform_Rotation);
         referenceFrameToBeCorrected.update();

         RigidBodyTransform startPoseTransform = new RigidBodyTransform();
         Vector3D startPose_Translation = new Vector3D(0.0, 0.0, 0.0);
         Quaternion startPose_Rotation = new Quaternion(0.0,0.0,0.0,1.0);

         startPoseTransform.setIdentity();
         startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
         startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), startPose_Translation));
         startPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
         startPoseTransform.multiply(new RigidBodyTransform(startPose_Rotation, new Vector3D()));

         FramePose3D startPose = new FramePose3D(worldFrame);
         startPose.set(startPoseTransform);

         RigidBodyTransform goalPoseTransform = new RigidBodyTransform();
         Vector3D goalPose_Translation = new Vector3D(0.0, 0.0, 0.0);
         Quaternion goalPose_Rotation = new Quaternion();//RandomTools.generateRandomQuaternion(random, 0.01);
         goalPose_Rotation.setYawPitchRoll(0.02, 0.0, 0.0);

         //saves the yaw for the assert
         Quaternion goalPose_RotationWithoutPitchAndRoll = new Quaternion();
         double[] goalPoseYawPitchRoll = new double[3];
         YawPitchRollConversion.convertQuaternionToYawPitchRoll(goalPose_Rotation, goalPoseYawPitchRoll);
         goalPose_RotationWithoutPitchAndRoll.setYawPitchRoll(goalPoseYawPitchRoll[0], 0.0, 0.0);

         goalPoseTransform.setIdentity();
         goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
         goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), goalPose_Translation));
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

         ////////////////update goalPose for asserts////
         referenceFrameToBeCorrectedTransform.getTranslation(referenceFrameToBeCorrectedTransform_Translation);
         referenceFrameToBeCorrectedTransform.getRotation(referenceFrameToBeCorrectedTransform_Rotation);


         FramePose3D expectedGoalPose = new FramePose3D(worldFrame);
         RigidBodyTransform expectedGoalPoseTransform = new RigidBodyTransform();
         expectedGoalPoseTransform.setIdentity();
         expectedGoalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
         expectedGoalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), goalPose_Translation));
         expectedGoalPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
         expectedGoalPoseTransform.multiply(new RigidBodyTransform(goalPose_RotationWithoutPitchAndRoll, new Vector3D()));
         expectedGoalPose.set(expectedGoalPoseTransform);

         assertTrue(interpolatedPose.epsilonEquals(expectedGoalPose, 1e-4));
      }
   }

   // TODO
   @Disabled
   @Test
   public void testTranslationAndRotationErrorsInterpolation()
   {
      Random random = new Random();
      int numberOfTicks = 2000;
      alphaFilterBreakFrequency.set(0.6);

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, referenceFrameToBeCorrected,
            alphaFilterBreakFrequency, dt, true);

      generateRandomReferenceFrameToBeCorrectedWaypoints(0, numberOfTicks);
      TimeStampedTransform3D temporaryTimeStampedTransform = new TimeStampedTransform3D();

      for (int i = 0; i < 1000; i++)
      {
         referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.findTransform(0, temporaryTimeStampedTransform);
         referenceFrameToBeCorrectedTransform = temporaryTimeStampedTransform.getTransform3D();
         referenceFrameToBeCorrectedTransform.getTranslation(referenceFrameToBeCorrectedTransform_Translation);
         referenceFrameToBeCorrectedTransform.getRotation(referenceFrameToBeCorrectedTransform_Rotation);
         referenceFrameToBeCorrected.update();

         RigidBodyTransform startPoseTransform = new RigidBodyTransform();
         Vector3D startPose_Translation = new Vector3D(0.0, 0.0, 0.0);
         Quaternion startPose_Rotation = new Quaternion(0.0,0.0,0.0,1.0);

         startPoseTransform.setIdentity();
         startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
         startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), startPose_Translation));
         startPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
         startPoseTransform.multiply(new RigidBodyTransform(startPose_Rotation, new Vector3D()));

         FramePose3D startPose = new FramePose3D(worldFrame);
         startPose.set(startPoseTransform);

         RigidBodyTransform goalPoseTransform = new RigidBodyTransform();
         Vector3D goalPose_Translation = RandomGeometry.nextVector3D(random, 0.04);
         Quaternion goalPose_Rotation = RandomGeometry.nextQuaternion(random, 0.04);

         goalPoseTransform.setIdentity();
         goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
         goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), goalPose_Translation));
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

         ////////////////update goalPose for asserts////
         referenceFrameToBeCorrectedTransform.getTranslation(referenceFrameToBeCorrectedTransform_Translation);
         referenceFrameToBeCorrectedTransform.getRotation(referenceFrameToBeCorrectedTransform_Rotation);

         goalPoseTransform.setIdentity();
         goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
         goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), goalPose_Translation));
         goalPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
         goalPoseTransform.multiply(new RigidBodyTransform(goalPose_Rotation, new Vector3D()));
         goalPose.set(goalPoseTransform);

         assertTrue(interpolatedPose.epsilonEquals(goalPose, 1e-4));
      }
   }

   @Test
   public void testMaxTranslationalCorrectionSpeedClip()
   {
      int numberOfTicks = 10000;
      alphaFilterBreakFrequency.set(0.6);

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, referenceFrameToBeCorrected,
            alphaFilterBreakFrequency, dt, false);


      generateRandomReferenceFrameToBeCorrectedWaypoints(0, numberOfTicks);
      TimeStampedTransform3D temporaryTimeStampedTransform = new TimeStampedTransform3D();

      referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.findTransform(0, temporaryTimeStampedTransform);
      referenceFrameToBeCorrectedTransform = temporaryTimeStampedTransform.getTransform3D();
      referenceFrameToBeCorrectedTransform.getTranslation(referenceFrameToBeCorrectedTransform_Translation);
      referenceFrameToBeCorrectedTransform.getRotation(referenceFrameToBeCorrectedTransform_Rotation);
      referenceFrameToBeCorrected.update();

      RigidBodyTransform startPoseTransform = new RigidBodyTransform();
      Vector3D startPose_Translation = new Vector3D(0.0, 0.0, 0.0);
      Quaternion startPose_Rotation = new Quaternion(0.0,0.0,0.0,1.0);

      startPoseTransform.setIdentity();
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), startPose_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
      startPoseTransform.multiply(new RigidBodyTransform(startPose_Rotation, new Vector3D()));

      FramePose3D startPose = new FramePose3D(worldFrame);
      startPose.set(startPoseTransform);

      RigidBodyTransform goalPoseTransform = new RigidBodyTransform();
      Vector3D goalPose_Translation = new Vector3D(1.0, 1.0, 1.0);
      Quaternion goalPose_Rotation = new Quaternion(0.0,0.0,0.0,1.0);

      goalPoseTransform.setIdentity();
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), goalPose_Translation));
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

         if (j>0 && (j%999) == 0)
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

   //TODO
   @Disabled
   @Test
   public void testMaxRotationalCorrectionSpeedClip()
   {
      int numberOfTicks = 10000;
      alphaFilterBreakFrequency.set(0.6);

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, referenceFrameToBeCorrected,
            alphaFilterBreakFrequency, dt, true);

      generateRandomReferenceFrameToBeCorrectedWaypoints(0, numberOfTicks);
      TimeStampedTransform3D temporaryTimeStampedTransform = new TimeStampedTransform3D();

      referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.findTransform(0, temporaryTimeStampedTransform);
      referenceFrameToBeCorrectedTransform = temporaryTimeStampedTransform.getTransform3D();
      referenceFrameToBeCorrectedTransform.getTranslation(referenceFrameToBeCorrectedTransform_Translation);
      referenceFrameToBeCorrectedTransform.getRotation(referenceFrameToBeCorrectedTransform_Rotation);
      referenceFrameToBeCorrected.update();

      RigidBodyTransform startPoseTransform = new RigidBodyTransform();
      Vector3D startPose_Translation = new Vector3D(0.0, 0.0, 0.0);
      Quaternion startPose_Rotation = new Quaternion(0.0,0.0,0.0,1.0);

      startPoseTransform.setIdentity();
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), startPose_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
      startPoseTransform.multiply(new RigidBodyTransform(startPose_Rotation, new Vector3D()));

      FramePose3D startPose = new FramePose3D(worldFrame);
      startPose.set(startPoseTransform);

      RigidBodyTransform goalPoseTransform = new RigidBodyTransform();
      Vector3D goalPose_Translation = new Vector3D(0.0, 0.0, 0.0);
      Quaternion goalPose_Rotation = new Quaternion(0.0,0.0,0.0,1.0);
      goalPose_Rotation.setYawPitchRoll(1.0, 1.0, 1.0);

      goalPoseTransform.setIdentity();
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), goalPose_Translation));
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

         if (j>0 && (j%999) == 0)
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

   // TODO
   @Disabled
   @Test
   public void testMaxCorrectionSpeedClipWorksWhenTranslationAndRotationOffsetsAreBig()
   {
      int numberOfTicks = 10000;
      alphaFilterBreakFrequency.set(0.6);

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, referenceFrameToBeCorrected,
            alphaFilterBreakFrequency, dt, true);

      generateRandomReferenceFrameToBeCorrectedWaypoints(0, numberOfTicks);
      TimeStampedTransform3D temporaryTimeStampedTransform = new TimeStampedTransform3D();

      referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.findTransform(0, temporaryTimeStampedTransform);
      referenceFrameToBeCorrectedTransform = temporaryTimeStampedTransform.getTransform3D();
      referenceFrameToBeCorrectedTransform.getTranslation(referenceFrameToBeCorrectedTransform_Translation);
      referenceFrameToBeCorrectedTransform.getRotation(referenceFrameToBeCorrectedTransform_Rotation);
      referenceFrameToBeCorrected.update();

      //test when rotation is bigger than translation
      RigidBodyTransform startPoseTransform = new RigidBodyTransform();
      Vector3D startPose_Translation = new Vector3D(0.0, 0.0, 0.0);
      Quaternion startPose_Rotation = new Quaternion(0.0,0.0,0.0,1.0);

      startPoseTransform.setIdentity();
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), startPose_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
      startPoseTransform.multiply(new RigidBodyTransform(startPose_Rotation, new Vector3D()));

      FramePose3D startPose = new FramePose3D(worldFrame);
      startPose.set(startPoseTransform);

      RigidBodyTransform goalPoseTransform = new RigidBodyTransform();
      Vector3D goalPose_Translation = new Vector3D(0.2, 0.2, 0.2);
      Quaternion goalPose_Rotation = new Quaternion(0.0,0.0,0.0,1.0);
      goalPose_Rotation.setYawPitchRoll(1.0, 1.0, 1.0);

      goalPoseTransform.setIdentity();
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), goalPose_Translation));
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

         if (j>0 && (j%999) == 0)
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

      //test when translation is bigger than rotation
      startPoseTransform = new RigidBodyTransform();
      startPose_Translation = new Vector3D(0.0, 0.0, 0.0);
      startPose_Rotation = new Quaternion(0.0,0.0,0.0,1.0);

      startPoseTransform.setIdentity();
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), startPose_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
      startPoseTransform.multiply(new RigidBodyTransform(startPose_Rotation, new Vector3D()));

      startPose = new FramePose3D(worldFrame);
      startPose.set(startPoseTransform);

      goalPoseTransform = new RigidBodyTransform();
      goalPose_Translation = new Vector3D(1.0, 1.0, 1.0);
      goalPose_Rotation = new Quaternion(0.0,0.0,0.0,1.0);
      goalPose_Rotation.setYawPitchRoll(0.2, 0.2, 0.2);

      goalPoseTransform.setIdentity();
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), goalPose_Translation));
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

         if (j>0 && (j%999) == 0)
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
      Random random = new Random();
      int numberOfTicks = 2000;
      alphaFilterBreakFrequency.set(0.6);

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, referenceFrameToBeCorrected,
            alphaFilterBreakFrequency, dt, false);

      generateRandomReferenceFrameToBeCorrectedWaypoints(0, numberOfTicks);
      TimeStampedTransform3D temporaryTimeStampedTransform = new TimeStampedTransform3D();

      for (int i = 0; i < 1000; i++)
      {
         referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.findTransform(0, temporaryTimeStampedTransform);
         referenceFrameToBeCorrectedTransform = temporaryTimeStampedTransform.getTransform3D();
         referenceFrameToBeCorrectedTransform.getTranslation(referenceFrameToBeCorrectedTransform_Translation);
         referenceFrameToBeCorrectedTransform.getRotation(referenceFrameToBeCorrectedTransform_Rotation);
         referenceFrameToBeCorrected.update();

         RigidBodyTransform startPoseTransform = new RigidBodyTransform();
         Vector3D startPose_Translation = new Vector3D(0.0, 0.0, 0.0);
         Quaternion startPose_Rotation = new Quaternion(0.0,0.0,0.0,1.0);

         startPoseTransform.setIdentity();
         startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
         startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), startPose_Translation));
         startPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
         startPoseTransform.multiply(new RigidBodyTransform(startPose_Rotation, new Vector3D()));

         FramePose3D startPose = new FramePose3D(worldFrame);
         startPose.set(startPoseTransform);

         RigidBodyTransform goalPoseTransform = new RigidBodyTransform();
         Vector3D goalPose_Translation = new Vector3D(0.0, 0.0, 0.0);
         Quaternion goalPose_Rotation = RandomGeometry.nextQuaternion(random, 0.04);

         goalPoseTransform.setIdentity();
         goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
         goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), goalPose_Translation));
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
         referenceFrameToBeCorrectedTransform.getTranslation(referenceFrameToBeCorrectedTransform_Translation);
         referenceFrameToBeCorrectedTransform.getRotation(referenceFrameToBeCorrectedTransform_Rotation);

         startPoseTransform.setIdentity();
         startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), referenceFrameToBeCorrectedTransform_Translation));
         startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(0.0,0.0,0.0,1.0), startPose_Translation));
         startPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
         startPoseTransform.multiply(new RigidBodyTransform(startPose_Rotation, new Vector3D()));
         startPose.set(startPoseTransform);

         assertTrue(interpolatedPose.epsilonEquals(startPose, 1e-4));
      }
   }

   @Test
   public void testErrorRotationCheckIsBehavingProperly()
   {
      Random random = new Random();
      alphaFilterBreakFrequency.set(0.6);

      ClippedSpeedOffsetErrorInterpolator clippedSpeedOffsetErrorInterpolator = new ClippedSpeedOffsetErrorInterpolator(registry, referenceFrameToBeCorrected,
            alphaFilterBreakFrequency, dt, false);

      CorrectedPelvisPoseErrorTooBigChecker correctedPelvisPoseErrorTooBigChecker = new CorrectedPelvisPoseErrorTooBigChecker(registry);

      FramePose3D startPose = new FramePose3D(worldFrame, new Point3D(), new Quaternion(0.0,0.0,0.0,1.0));
      FramePose3D goalPose = new FramePose3D(worldFrame);
      Quaternion goalOrientation= new Quaternion();

      for(int i = 0; i < 200; i++)
      {
         goalPose.set(RandomGeometry.nextPoint3D(random, 0.1, 0.1, 0.1),RandomGeometry.nextQuaternion(random, Math.toRadians(9.2)));
         assertFalse(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));
      }

      goalOrientation.setYawPitchRoll(Math.toRadians(10.1), Math.toRadians(0.0), Math.toRadians(0.0));
      goalPose.set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(10.1), Math.toRadians(0.0));
      goalPose.set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(10.1));
      goalPose.set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(10.2), Math.toRadians(10.3), Math.toRadians(0.0));
      goalPose.set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(10.2), Math.toRadians(0.0), Math.toRadians(10.1));
      goalPose.set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(10.2), Math.toRadians(10.1));
      goalPose.set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(20.0), Math.toRadians(10.2), Math.toRadians(10.1));
      goalPose.set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(-10.1), Math.toRadians(0.0), Math.toRadians(0.0));
      goalPose.set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(-10.1), Math.toRadians(0.0));
      goalPose.set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(-10.1));
      goalPose.set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(-10.2), Math.toRadians(-10.3), Math.toRadians(0.0));
      goalPose.set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(-10.2), Math.toRadians(0.0), Math.toRadians(-10.1));
      goalPose.set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(-10.2), Math.toRadians(-10.1));
      goalPose.set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));

      goalOrientation.setYawPitchRoll(Math.toRadians(-20.0), Math.toRadians(-10.2), Math.toRadians(-10.1));
      goalPose.set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0), goalOrientation);
      assertTrue(correctedPelvisPoseErrorTooBigChecker.checkIfErrorIsTooBig(startPose, goalPose, true));


   }


   private void generateRandomReferenceFrameToBeCorrectedWaypoints(long firstTimestamp, long lastTimestamp)
   {
      Random random = new Random();
      Vector3D translation = new Vector3D();
      Quaternion rotation = new Quaternion(0.0,0.0,0.0,1.0);

      translation = RandomGeometry.nextVector3D(random);
      rotation = RandomGeometry.nextQuaternion(random);
      putReferenceFrameToBeCorrectedWaypointInTransformBuffer(firstTimestamp, translation, rotation);

      translation = RandomGeometry.nextVector3D(random);
      rotation = RandomGeometry.nextQuaternion(random);
      putReferenceFrameToBeCorrectedWaypointInTransformBuffer(lastTimestamp, translation, rotation);
   }

   private void putReferenceFrameToBeCorrectedWaypointInTransformBuffer(long timeStamp, Vector3D translation, Quaternion rotation)
   {
      RigidBodyTransform temporaryReferenceFrameToBeCorrectedTransformInWorldFrame = new RigidBodyTransform();
      temporaryReferenceFrameToBeCorrectedTransformInWorldFrame.setTranslation(translation);
      temporaryReferenceFrameToBeCorrectedTransformInWorldFrame.setRotation(rotation);
      referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.put(temporaryReferenceFrameToBeCorrectedTransformInWorldFrame, timeStamp);
   }

}
