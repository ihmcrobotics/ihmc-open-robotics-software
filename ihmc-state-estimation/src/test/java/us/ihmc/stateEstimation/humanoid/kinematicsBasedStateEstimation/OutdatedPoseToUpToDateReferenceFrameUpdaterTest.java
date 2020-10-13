package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.subscribers.TimeStampedTransformBuffer;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class OutdatedPoseToUpToDateReferenceFrameUpdaterTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testGetUpToDateTimeStampedBufferNewestTimeStamp()
   {
      FramePose3D upToDatePoseInPresent = new FramePose3D(worldFrame);
      PoseReferenceFrame upToDateReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", upToDatePoseInPresent);
      int numberOfUpToDateTransforms = 20;
      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(
            numberOfUpToDateTransforms, upToDateReferenceFrameInPresent);

      Random random = new Random(42L);
      long timeStamp;
      RigidBodyTransform transform;

      for (int i = 0; i < 100; i++)
      {
         timeStamp = random.nextLong();
         transform = generateRandomUpToDateTransforms(random);
         outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(transform, timeStamp);
         assertTrue(timeStamp == outdatedPoseToUpToDateReferenceFrameUpdater.getStateEstimatorTimeStampedBufferNewestTimestamp());
      }
   }

   @Test
   public void testGetUpToDateTimeStampedBufferOldestTimeStamp()
   {
      FramePose3D upToDatePoseInPresent = new FramePose3D(worldFrame);
      PoseReferenceFrame upToDateReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", upToDatePoseInPresent);
      int numberOfUpToDateTransforms = 20;
      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(
            numberOfUpToDateTransforms, upToDateReferenceFrameInPresent);

      Random random = new Random(42L);
      long timeStamp = 100;
      long firstTimeStamp = timeStamp;
      RigidBodyTransform transform;

      for (int i = 0; i < 100; i++)
      {
         transform = generateRandomUpToDateTransforms(random);
         outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(transform, timeStamp);
         if (i < 20)
            assertTrue(outdatedPoseToUpToDateReferenceFrameUpdater.getStateEstimatorTimeStampedBufferOldestTimestamp() == firstTimeStamp);
         else
            assertTrue((timeStamp - (numberOfUpToDateTransforms - 1)) == outdatedPoseToUpToDateReferenceFrameUpdater.getStateEstimatorTimeStampedBufferOldestTimestamp());
         timeStamp++;
      }
   }
   
   @Test
   public void testComputedRotationError()
   {
      Random random = new Random(1987L);
      int numberOfUpToDateTransforms = 1000;
      int numberOfOutdatedTransforms = numberOfUpToDateTransforms / 2;
      
      Vector3D[] translationOffsets = new Vector3D[numberOfOutdatedTransforms];
      Quaternion[] orientationOffsets = new Quaternion[numberOfOutdatedTransforms];
      long[] outdatedTimeStamps = new long[numberOfOutdatedTransforms];
      
      FramePose3D upToDatePoseInPresent = new FramePose3D(worldFrame);
      PoseReferenceFrame stateEsimatorReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", upToDatePoseInPresent);
      
      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(
            numberOfUpToDateTransforms, stateEsimatorReferenceFrameInPresent);
      
      TimeStampedTransformBuffer upToDateTimeStampedTransformPoseBuffer = new TimeStampedTransformBuffer(numberOfUpToDateTransforms);
      TimeStampedTransformBuffer outdatedTimeStampedTransformBuffer = new TimeStampedTransformBuffer(numberOfOutdatedTransforms);
      
      for (int timeStamp = 0; timeStamp < numberOfUpToDateTransforms; timeStamp++)
      {
         RigidBodyTransform upToDateTransform = generateRandomUpToDateTransforms(random);
         upToDateTimeStampedTransformPoseBuffer.put(upToDateTransform, timeStamp);
         outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(upToDateTransform, timeStamp);
      }
      
      //generate outdatedTransforms offsets based on the upToDateTransforms
      for (int j = 0; j < numberOfOutdatedTransforms; j++)
      {
         int timeStamp = j * 2;
         outdatedTimeStamps[j] = timeStamp;
         translationOffsets[j] = new Vector3D();//RandomTools.generateRandomVector(random, -2.0, -2.0, 0.0, 2.0, 2.0, 2.0);
         orientationOffsets[j] = RandomGeometry.nextQuaternion(random, Math.PI / 2.0);

         RigidBodyTransform outdatedTransform = generateOutdatedTransformWithTranslationAndOrientationOffset(upToDateTimeStampedTransformPoseBuffer, timeStamp,
               orientationOffsets[j], translationOffsets[j]);

         outdatedTimeStampedTransformBuffer.put(outdatedTransform, timeStamp);
      }
      
      for(int i = 0; i < numberOfOutdatedTransforms; i++)
      {
         int timeStamp = i * 2;
         TimeStampedTransform3D localizationTimeStampedTransformInWorld = new TimeStampedTransform3D();
         outdatedTimeStampedTransformBuffer.findTransform(timeStamp, localizationTimeStampedTransformInWorld);
         outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(localizationTimeStampedTransformInWorld);
         
         RigidBodyTransform totalError = new RigidBodyTransform();
         outdatedPoseToUpToDateReferenceFrameUpdater.getTotalErrorTransform(totalError);
         Quaternion calculatedRotationError = new Quaternion();
         calculatedRotationError.set(totalError.getRotation());
         Quaternion actualError = orientationOffsets[i];
         
         assertTrue(calculatedRotationError.epsilonEquals(actualError, 1e-4));
      }
   }
   
   @Test
   public void testComputedTranslationError()
   {
      Random random = new Random(1987L);
      int numberOfUpToDateTransforms = 1000;
      int numberOfOutdatedTransforms = numberOfUpToDateTransforms / 2;
      
      Vector3D[] translationOffsets = new Vector3D[numberOfOutdatedTransforms];
      Quaternion[] orientationOffsets = new Quaternion[numberOfOutdatedTransforms];
      long[] outdatedTimeStamps = new long[numberOfOutdatedTransforms];
      
      FramePose3D upToDatePoseInPresent = new FramePose3D(worldFrame);
      PoseReferenceFrame stateEsimatorReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", upToDatePoseInPresent);
      
      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(
            numberOfUpToDateTransforms, stateEsimatorReferenceFrameInPresent);
      
      TimeStampedTransformBuffer upToDateTimeStampedTransformPoseBuffer = new TimeStampedTransformBuffer(numberOfUpToDateTransforms);
      TimeStampedTransformBuffer outdatedTimeStampedTransformBuffer = new TimeStampedTransformBuffer(numberOfOutdatedTransforms);
      
      for (int timeStamp = 0; timeStamp < numberOfUpToDateTransforms; timeStamp++)
      {
         RigidBodyTransform upToDateTransform = generateRandomUpToDateTransforms(random);
         upToDateTimeStampedTransformPoseBuffer.put(upToDateTransform, timeStamp);
         outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(upToDateTransform, timeStamp);
      }
      
      //generate outdatedTransforms offsets based on the upToDateTransforms
      for (int j = 0; j < numberOfOutdatedTransforms; j++)
      {
         int timeStamp = j * 2;
         outdatedTimeStamps[j] = timeStamp;
         translationOffsets[j] = RandomGeometry.nextVector3D(random, -2.0, -2.0, 0.0, 2.0, 2.0, 2.0);
         orientationOffsets[j] = RandomGeometry.nextQuaternion(random, Math.PI / 2.0);
         
         RigidBodyTransform outdatedTransform = generateOutdatedTransformWithTranslationAndOrientationOffset(upToDateTimeStampedTransformPoseBuffer, timeStamp,
               orientationOffsets[j], translationOffsets[j]);
         
         outdatedTimeStampedTransformBuffer.put(outdatedTransform, timeStamp);
      }
      
      for(int i = 0; i < numberOfOutdatedTransforms; i++)
      {
         int timeStamp = i * 2;
         TimeStampedTransform3D localizationTimeStampedTransformInWorld = new TimeStampedTransform3D();
         outdatedTimeStampedTransformBuffer.findTransform(timeStamp, localizationTimeStampedTransformInWorld);
         outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(localizationTimeStampedTransformInWorld);
         
         RigidBodyTransform totalError = new RigidBodyTransform();
         outdatedPoseToUpToDateReferenceFrameUpdater.getTotalErrorTransform(totalError);
         
         Point3D calculatedTranslationError = new Point3D();
         calculatedTranslationError.set(totalError.getTranslation());
         Vector3D actualTranslationError = translationOffsets[i];
         
         Quaternion calculatedRotationError = new Quaternion();
         calculatedRotationError.set(totalError.getRotation());
         Quaternion actualRotationError = orientationOffsets[i];
         
         assertTrue(calculatedTranslationError.epsilonEquals(actualTranslationError, 1e-4));
         assertTrue(calculatedRotationError.epsilonEquals(actualRotationError, 1e-4));
      }
   }
   
   @Test
   public void testComputedError()
   {
      Random random = new Random(1987L);
      int numberOfUpToDateTransforms = 1000;
      int numberOfOutdatedTransforms = numberOfUpToDateTransforms / 2;
      
      Vector3D[] translationOffsets = new Vector3D[numberOfOutdatedTransforms];
      Quaternion[] orientationOffsets = new Quaternion[numberOfOutdatedTransforms];
      long[] outdatedTimeStamps = new long[numberOfOutdatedTransforms];
      
      FramePose3D upToDatePoseInPresent = new FramePose3D(worldFrame);
      PoseReferenceFrame stateEsimatorReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", upToDatePoseInPresent);
      
      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(
            numberOfUpToDateTransforms, stateEsimatorReferenceFrameInPresent);
      
      TimeStampedTransformBuffer upToDateTimeStampedTransformPoseBuffer = new TimeStampedTransformBuffer(numberOfUpToDateTransforms);
      TimeStampedTransformBuffer outdatedTimeStampedTransformBuffer = new TimeStampedTransformBuffer(numberOfOutdatedTransforms);
      
      for (int timeStamp = 0; timeStamp < numberOfUpToDateTransforms; timeStamp++)
      {
         RigidBodyTransform upToDateTransform = generateRandomUpToDateTransforms(random);
         upToDateTimeStampedTransformPoseBuffer.put(upToDateTransform, timeStamp);
         outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(upToDateTransform, timeStamp);
      }
      
      //generate outdatedTransforms offsets based on the upToDateTransforms
      for (int j = 0; j < numberOfOutdatedTransforms; j++)
      {
         int timeStamp = j * 2;
         outdatedTimeStamps[j] = timeStamp;
         translationOffsets[j] = RandomGeometry.nextVector3D(random, -2.0, -2.0, 0.0, 2.0, 2.0, 2.0);
         orientationOffsets[j] = RandomGeometry.nextQuaternion(random, Math.PI / 2.0);
         
         RigidBodyTransform outdatedTransform = generateOutdatedTransformWithTranslationAndOrientationOffset(upToDateTimeStampedTransformPoseBuffer, timeStamp,
               orientationOffsets[j], translationOffsets[j]);
         
         outdatedTimeStampedTransformBuffer.put(outdatedTransform, timeStamp);
      }
      
      for(int i = 0; i < numberOfOutdatedTransforms; i++)
      {
         int timeStamp = i * 2;
         TimeStampedTransform3D localizationTimeStampedTransformInWorld = new TimeStampedTransform3D();
         outdatedTimeStampedTransformBuffer.findTransform(timeStamp, localizationTimeStampedTransformInWorld);
         outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(localizationTimeStampedTransformInWorld);
         
         RigidBodyTransform totalError = new RigidBodyTransform();
         outdatedPoseToUpToDateReferenceFrameUpdater.getTotalErrorTransform(totalError);
         Point3D calculatedTranslationError = new Point3D();
         calculatedTranslationError.set(totalError.getTranslation());
         Vector3D actualError = translationOffsets[i];
         
         assertTrue(calculatedTranslationError.epsilonEquals(actualError, 1e-4));
      }
   }
   
   @Test
   public void testSimpleTranslationAtKnownLocation()
   {
      Random random = new Random(1987L);
      FramePose3D upToDatePoseInPresent = new FramePose3D(worldFrame);
      PoseReferenceFrame stateEsimatorReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", upToDatePoseInPresent);

      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(10,
            stateEsimatorReferenceFrameInPresent);

      outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(new RigidBodyTransform(), 1);

      RigidBodyTransform localizationRigidBody = new RigidBodyTransform(new Quaternion(), new Point3D(1.0, 1.0, 1.0));
      FramePose3D expectedPose = new FramePose3D(worldFrame);
      expectedPose.getPosition().set(1.0, 1.0, 1.0);
      
      TimeStampedTransform3D localizationTimeStampedTransformInWorld = new TimeStampedTransform3D(localizationRigidBody, 1);
      outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(localizationTimeStampedTransformInWorld);
      
      ReferenceFrame localizationReferenceFrameToBeUpdated = outdatedPoseToUpToDateReferenceFrameUpdater.getLocalizationReferenceFrameToBeUpdated();
      
      FramePose3D calculatedPose = new FramePose3D(localizationReferenceFrameToBeUpdated);
      calculatedPose.changeFrame(worldFrame);
      
      assertTrue(calculatedPose.epsilonEquals(expectedPose, 1e-4));

   }
   
   @Test
   public void testNoDifferenceBetweenStateEstimatorAndLocalization()
   {
      FramePose3D upToDatePoseInPresent = new FramePose3D(worldFrame);
      upToDatePoseInPresent.getPosition().set(1.0, 1.0, 1.0);
      upToDatePoseInPresent.getOrientation().setYawPitchRoll(Math.PI/8, Math.PI/8, Math.PI/8);
      
      FramePose3D expectedPose = new FramePose3D(worldFrame);
      expectedPose.getOrientation().setYawPitchRoll(Math.PI/8, Math.PI/8, Math.PI/8);
      expectedPose.getPosition().set(1.0, 1.0, 1.0);
      
      RigidBodyTransform stateEstimatorRigidBody = new RigidBodyTransform();
      RigidBodyTransform localizationRigidBody = new RigidBodyTransform();
      
      upToDatePoseInPresent.get(stateEstimatorRigidBody);
      upToDatePoseInPresent.get(localizationRigidBody);
      
      PoseReferenceFrame stateEsimatorReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", upToDatePoseInPresent);
      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(10,
            stateEsimatorReferenceFrameInPresent);
      
      outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(stateEstimatorRigidBody, 1);
      
      TimeStampedTransform3D localizationTimeStampedTransformInWorld = new TimeStampedTransform3D(localizationRigidBody, 1);
      outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(localizationTimeStampedTransformInWorld);
      
      ReferenceFrame localizationReferenceFrameToBeUpdated = outdatedPoseToUpToDateReferenceFrameUpdater.getLocalizationReferenceFrameToBeUpdated();
      
      FramePose3D calculatedPose = new FramePose3D(localizationReferenceFrameToBeUpdated);
      calculatedPose.changeFrame(worldFrame);
      
      assertTrue(calculatedPose.epsilonEquals(expectedPose, 1e-4));
   }
   
   //this tests fails, I don't think OutdatedPoseToUpToDateReferenceFrameUpdater can support more than a single rotation at a time
   @Disabled
   @Test
   public void testKnownDifferenceBetweenStateEstimatorAndLocalization()
   {
      FramePose3D stateEstimatorPresent = new FramePose3D(worldFrame);
      stateEstimatorPresent.getPosition().set(2.0, 22.0, 1.0);
      stateEstimatorPresent.getOrientation().setYawPitchRoll(Math.PI, Math.PI / 32.0, Math.PI / 16.0);
      PoseReferenceFrame stateEsimatorReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", stateEstimatorPresent);

      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(10,
            stateEsimatorReferenceFrameInPresent);

      //create pose in past
      RigidBodyTransform stateEstimatorRigidBodyTransform = new RigidBodyTransform();
      FramePose3D stateEstimatorInPast = new FramePose3D(worldFrame);
      stateEstimatorInPast.getPosition().set(1.0, 20.0, 0.8);
      stateEstimatorInPast.getOrientation().setYawPitchRoll(Math.PI - (Math.PI / 64.0), Math.PI / 64.0, Math.PI / 8.0);
      stateEstimatorInPast.get(stateEstimatorRigidBodyTransform);
      outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(stateEstimatorRigidBodyTransform, 1);

      //create localization pose in past
      RigidBodyTransform localizationRigidBody = new RigidBodyTransform();
      FramePose3D localizationInPast = new FramePose3D(worldFrame);
      localizationInPast.getPosition().set(1.5, 21.8, 1.1);
      localizationInPast.getOrientation().setYawPitchRoll(Math.PI - (Math.PI / 32.0), Math.PI / 16.0, Math.PI / 4);
      localizationInPast.get(localizationRigidBody);
      
      TimeStampedTransform3D localizationTimeStampedTransformInWorld = new TimeStampedTransform3D(localizationRigidBody, 1);
      outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(localizationTimeStampedTransformInWorld);
      
      ReferenceFrame localizationReferenceFrameToBeUpdated = outdatedPoseToUpToDateReferenceFrameUpdater.getLocalizationReferenceFrameToBeUpdated();
      FramePose3D calculatedPose = new FramePose3D(localizationReferenceFrameToBeUpdated);
      calculatedPose.changeFrame(worldFrame);
      
      FramePose3D expectedPose = new FramePose3D(worldFrame);
      expectedPose.getOrientation().setYawPitchRoll(Math.PI - (Math.PI / 64.0), (5.0 * Math.PI) / 64.0, (3.0 * Math.PI) / 16.0);
      expectedPose.getPosition().set(2.5, 23.8, 1.3);
      System.out.println("z: " + -Math.PI / 64.0 + " y: " + (3.0 * Math.PI) / 64.0 + " x: " + (Math.PI / 8.0));
      System.out.println(calculatedPose);
      System.out.println(expectedPose);
      assertTrue(calculatedPose.epsilonEquals(expectedPose, 1e-4));
   }
   
   @Test
   public void testKnownTranslationAndYawDifferenceBetweenStateEstimatorAndLocalization()
   {
      FramePose3D stateEstimatorPresent = new FramePose3D(worldFrame);
      stateEstimatorPresent.getPosition().set(2.0, 22.0, 1.0);
      stateEstimatorPresent.getOrientation().setYawPitchRoll(Math.PI, Math.PI / 64.0, Math.PI / 8.0);
      PoseReferenceFrame stateEsimatorReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", stateEstimatorPresent);
      
      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(10,
            stateEsimatorReferenceFrameInPresent);
      
      //create pose in past
      RigidBodyTransform stateEstimatorRigidBodyTransform = new RigidBodyTransform();
      FramePose3D stateEstimatorInPast = new FramePose3D(worldFrame);
      stateEstimatorInPast.getPosition().set(1.0, 20.0, 0.8);
      stateEstimatorInPast.getOrientation().setYawPitchRoll(Math.PI - (Math.PI / 64.0), Math.PI / 64.0, Math.PI / 8.0);
      stateEstimatorInPast.get(stateEstimatorRigidBodyTransform);
      outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(stateEstimatorRigidBodyTransform, 1);
      
      //create localization pose in past
      RigidBodyTransform localizationRigidBody = new RigidBodyTransform();
      FramePose3D localizationInPast = new FramePose3D(worldFrame);
      localizationInPast.getPosition().set(1.5, 21.8, 1.1);
      localizationInPast.getOrientation().setYawPitchRoll(Math.PI - (Math.PI / 32.0), Math.PI / 64.0, Math.PI / 8.0);
      localizationInPast.get(localizationRigidBody);
      
      TimeStampedTransform3D localizationTimeStampedTransformInWorld = new TimeStampedTransform3D(localizationRigidBody, 1);
      outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(localizationTimeStampedTransformInWorld);
      
      ReferenceFrame localizationReferenceFrameToBeUpdated = outdatedPoseToUpToDateReferenceFrameUpdater.getLocalizationReferenceFrameToBeUpdated();
      FramePose3D calculatedPose = new FramePose3D(localizationReferenceFrameToBeUpdated);
      calculatedPose.changeFrame(worldFrame);
      
      FramePose3D expectedPose = new FramePose3D(worldFrame);
      expectedPose.getOrientation().setYawPitchRoll(Math.PI - (Math.PI / 64.0), Math.PI / 64.0, Math.PI / 8.0);
      expectedPose.getPosition().set(2.5, 23.8, 1.3);
      System.out.println("z: " + -Math.PI / 64.0 + " y: " + 0.0 + " x: " + 0.0);
      System.out.println(calculatedPose);
      System.out.println(expectedPose);
      assertTrue(calculatedPose.epsilonEquals(expectedPose, 1e-4));
   }
   
   //this tests fails, I don't think OutdatedPoseToUpToDateReferenceFrameUpdater can support more than a single rotation at a time
   @Disabled
   @Test
   public void testKnownTranslationYawAndPitchDifferenceBetweenStateEstimatorAndLocalization()
   {
      FramePose3D stateEstimatorPresent = new FramePose3D(worldFrame);
      stateEstimatorPresent.getPosition().set(2.0, 22.0, 1.0);
      stateEstimatorPresent.getOrientation().setYawPitchRoll(Math.PI, Math.PI / 8.0, Math.PI / 8.0);
      PoseReferenceFrame stateEsimatorReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", stateEstimatorPresent);
      
      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(10,
            stateEsimatorReferenceFrameInPresent);
      
      //create pose in past
      RigidBodyTransform stateEstimatorRigidBodyTransform = new RigidBodyTransform();
      FramePose3D stateEstimatorInPast = new FramePose3D(worldFrame);
      stateEstimatorInPast.getPosition().set(1.0, 20.0, 0.8);
      stateEstimatorInPast.getOrientation().setYawPitchRoll(Math.PI - (Math.PI / 64.0), Math.PI / 64.0, Math.PI / 8.0);
      stateEstimatorInPast.get(stateEstimatorRigidBodyTransform);
      outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(stateEstimatorRigidBodyTransform, 1);
      
      //create localization pose in past
      RigidBodyTransform localizationRigidBody = new RigidBodyTransform();
      FramePose3D localizationInPast = new FramePose3D(worldFrame);
      localizationInPast.getPosition().set(1.5, 21.8, 1.1);
      localizationInPast.getOrientation().setYawPitchRoll(Math.PI - (Math.PI / 32.0), Math.PI / 32.0, Math.PI / 8.0);
      localizationInPast.get(localizationRigidBody);
      
      TimeStampedTransform3D localizationTimeStampedTransformInWorld = new TimeStampedTransform3D(localizationRigidBody, 1);
      outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(localizationTimeStampedTransformInWorld);
      
      ReferenceFrame localizationReferenceFrameToBeUpdated = outdatedPoseToUpToDateReferenceFrameUpdater.getLocalizationReferenceFrameToBeUpdated();
      FramePose3D calculatedPose = new FramePose3D(localizationReferenceFrameToBeUpdated);
      calculatedPose.changeFrame(worldFrame);
      
      FramePose3D expectedPose = new FramePose3D(worldFrame);
      expectedPose.getOrientation().setYawPitchRoll(Math.PI - (Math.PI / 64.0), 9.0 * Math.PI / 64.0, Math.PI / 8.0);
      expectedPose.getPosition().set(2.5, 23.8, 1.3);
      System.out.println("z: " + -Math.PI / 64.0 + " y: " + Math.PI / 32.0 + " x: " + 0.0);
      System.out.println(calculatedPose);
      System.out.println(expectedPose);
      assertTrue(calculatedPose.epsilonEquals(expectedPose, 1e-4));
   }
   
   @Test
   public void testSimpleRotationAtKnownLocation()
   {
      Random random = new Random(1987L);
      FramePose3D upToDatePoseInPresent = new FramePose3D(worldFrame);
      PoseReferenceFrame stateEsimatorReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", upToDatePoseInPresent);

      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(10,
            stateEsimatorReferenceFrameInPresent);

      outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(new RigidBodyTransform(), 1);

      
      RigidBodyTransform localizationRigidBody = new RigidBodyTransform();
      localizationRigidBody.setRotationEulerAndZeroTranslation(Math.PI/8, Math.PI/8, Math.PI/8);
      FramePose3D expectedPose = new FramePose3D(worldFrame);
      expectedPose.getOrientation().setYawPitchRoll(Math.PI/8, Math.PI/8, Math.PI/8);
      
      TimeStampedTransform3D localizationTimeStampedTransformInWorld = new TimeStampedTransform3D(localizationRigidBody, 1);
      outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(localizationTimeStampedTransformInWorld);
      
      ReferenceFrame localizationReferenceFrameToBeUpdated = outdatedPoseToUpToDateReferenceFrameUpdater.getLocalizationReferenceFrameToBeUpdated();
      
      FramePose3D calculatedPose = new FramePose3D(localizationReferenceFrameToBeUpdated);
      calculatedPose.changeFrame(worldFrame);
      
      assertTrue(calculatedPose.epsilonEquals(expectedPose, 1e-4));

   }

   //this tests fails, I don't think OutdatedPoseToUpToDateReferenceFrameUpdater can support more than a single rotation at a time
   @Disabled
   @Test
   public void testUpdateOutdatedTransformWithKnownOffsets()
   {
      int numberOfUpToDateTransforms = 10;
      int numberOfOutdatedTransforms = 3;
      long firstTimeStamp = 1000;
      long lastTimeStamp = 2000;
      int numberOfTicksOfDelay = 100;
      Random random = new Random(1987L);

      Vector3D[] translationOffsets = new Vector3D[numberOfOutdatedTransforms];
      Quaternion[] orientationOffsets = new Quaternion[numberOfOutdatedTransforms];
      long[] outdatedTimeStamps = new long[numberOfOutdatedTransforms];

      FramePose3D upToDatePoseInPresent = new FramePose3D(worldFrame);
      PoseReferenceFrame upToDateReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", upToDatePoseInPresent);
      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(
            numberOfUpToDateTransforms, upToDateReferenceFrameInPresent);
      ReferenceFrame outdatedReferenceFrame_ToBeUpdated;
      outdatedReferenceFrame_ToBeUpdated = outdatedPoseToUpToDateReferenceFrameUpdater.getLocalizationReferenceFrameToBeUpdated();
      
      TimeStampedTransformBuffer upToDateTimeStampedTransformPoseBuffer = new TimeStampedTransformBuffer(numberOfUpToDateTransforms);
      TimeStampedTransformBuffer outdatedTimeStampedTransformBuffer = new TimeStampedTransformBuffer(numberOfOutdatedTransforms);

      //generate uptoDateTransforms used later in the test as waypoints
      for (int i = 0; i < numberOfUpToDateTransforms; i++)
      {
         long timeStamp = (long) (i * (lastTimeStamp - firstTimeStamp) / numberOfUpToDateTransforms + firstTimeStamp);
         RigidBodyTransform upToDateTransform = generateRandomUpToDateTransforms(random);
         upToDateTimeStampedTransformPoseBuffer.put(upToDateTransform, timeStamp);
         outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(upToDateTransform, timeStamp);
      }
      
      //generate outdatedTransforms offsets based on the upToDateTransforms
      for (int j = 0; j < numberOfOutdatedTransforms; j++)
      {
         long timeStamp = (long) (j * (lastTimeStamp * 0.8 - firstTimeStamp * 0.8) / numberOfOutdatedTransforms + firstTimeStamp * 1.2);
         outdatedTimeStamps[j] = timeStamp;

         translationOffsets[j] = RandomGeometry.nextVector3D(random, -2.0, -2.0, 0.0, 2.0, 2.0, 2.0);
         orientationOffsets[j] = RandomGeometry.nextQuaternion(random, Math.PI / 2.0);//RandomTools.generateRandomQuaternion(random, Math.PI);

         RigidBodyTransform outdatedTransform = generateOutdatedTransformWithTranslationAndOrientationOffset(upToDateTimeStampedTransformPoseBuffer, timeStamp,
               orientationOffsets[j], translationOffsets[j]);

         outdatedTimeStampedTransformBuffer.put(outdatedTransform, timeStamp);
      }

      int outdatedTimeStampsIndex = -1;
      for (long timeStamp = firstTimeStamp; timeStamp < lastTimeStamp; timeStamp++)
      {
         //////////////////  update the upToDate referenceFrame
         TimeStampedTransform3D upToDateTimeStampedTransform = new TimeStampedTransform3D();
         upToDateTimeStampedTransformPoseBuffer.findTransform(timeStamp, upToDateTimeStampedTransform);
         upToDatePoseInPresent.set(upToDateTimeStampedTransform.getTransform3D());
         upToDateReferenceFrameInPresent.setPoseAndUpdate(upToDatePoseInPresent);
         //////////////////
         if (outdatedTimeStamps[0] == timeStamp - numberOfTicksOfDelay)
            outdatedTimeStampsIndex++;
         if (outdatedTimeStampsIndex >= 0)
         {
            if (outdatedTimeStampsIndex < numberOfOutdatedTransforms && outdatedTimeStamps[outdatedTimeStampsIndex] == timeStamp - numberOfTicksOfDelay)
            {
               TimeStampedTransform3D outdatedTimeStampedTransform = new TimeStampedTransform3D();
               outdatedTimeStampedTransformBuffer.findTransform(outdatedTimeStamps[outdatedTimeStampsIndex], outdatedTimeStampedTransform);
               assertTrue(outdatedPoseToUpToDateReferenceFrameUpdater.stateEstimatorTimeStampedBufferIsInRange(outdatedTimeStampedTransform.getTimeStamp()));
               outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(outdatedTimeStampedTransform);
               outdatedTimeStampsIndex++;
            }
         }
         outdatedReferenceFrame_ToBeUpdated.update();
         FramePose3D outdatedPoseUpdatedInWorldFrame = new FramePose3D(outdatedReferenceFrame_ToBeUpdated);
         outdatedPoseUpdatedInWorldFrame.changeFrame(worldFrame);

         Vector3D upToDateReferenceFrameInPresent_Translation = new Vector3D(upToDatePoseInPresent.getPosition());
         Vector3D outdatedPoseUpdatedInWorldFrame_Translation = new Vector3D(outdatedPoseUpdatedInWorldFrame.getPosition());
         
         FramePose3D testedPose = new FramePose3D(worldFrame);
         testedPose.set(outdatedPoseUpdatedInWorldFrame);
         testedPose.changeFrame(upToDateReferenceFrameInPresent);

         Vector3D testedTranslation = new Vector3D();
         testedTranslation.sub(outdatedPoseUpdatedInWorldFrame_Translation, upToDateReferenceFrameInPresent_Translation);
         Quaternion testedOrientation = new Quaternion(testedPose.getOrientation());

         if (timeStamp < (int) (firstTimeStamp * 1.2 + numberOfTicksOfDelay))
         {
            assertTrue(testedOrientation.epsilonEquals(new Quaternion(0.0, 0.0, 0.0, 1.0), 1e-4));
            assertTrue(testedTranslation.epsilonEquals(new Vector3D(0.0, 0.0, 0.0), 1e-8));
         }
         else
         {
            assertTrue(testedTranslation.epsilonEquals(translationOffsets[outdatedTimeStampsIndex - 1], 1e-8));
            assertTrue(testedOrientation.epsilonEquals(orientationOffsets[outdatedTimeStampsIndex - 1], 1e-4));
         }
      }
   }

   private RigidBodyTransform generateOutdatedTransformWithTranslationAndOrientationOffset(TimeStampedTransformBuffer upToDateTimeStampedTransformPoseBuffer,
         long timeStamp, Quaternion orientationOffset, Vector3D translationOffset)
   {
      TimeStampedTransform3D upToDateTimeStampedTransformInPast = new TimeStampedTransform3D();
      upToDateTimeStampedTransformPoseBuffer.findTransform(timeStamp, upToDateTimeStampedTransformInPast);
      
      RigidBodyTransform upToDateTransformInPast_Translation = new RigidBodyTransform(upToDateTimeStampedTransformInPast.getTransform3D());
      RigidBodyTransform upToDateTransformInPast_Rotation = new RigidBodyTransform(upToDateTransformInPast_Translation);
      upToDateTransformInPast_Translation.getRotation().setToZero();
      upToDateTransformInPast_Rotation.getTranslation().setToZero();
      
      RigidBodyTransform offsetRotationTransform = new RigidBodyTransform(orientationOffset, new Vector3D());
      RigidBodyTransform offsetTranslationTransform = new RigidBodyTransform(new Quaternion(), translationOffset);
      RigidBodyTransform transformedOutdatedTransform = new RigidBodyTransform();

      transformedOutdatedTransform.multiply(upToDateTransformInPast_Translation);
      transformedOutdatedTransform.multiply(offsetTranslationTransform);
      transformedOutdatedTransform.multiply(upToDateTransformInPast_Rotation);
      transformedOutdatedTransform.multiply(offsetRotationTransform);
      
      return transformedOutdatedTransform;
   }

   private RigidBodyTransform generateRandomUpToDateTransforms(Random random)
   {
      RigidBodyTransform upToDateTransform = new RigidBodyTransform();
      upToDateTransform.getTranslation().set(RandomGeometry.nextVector3D(random));
      upToDateTransform.getRotation().set(RandomGeometry.nextQuaternion(random));
      return upToDateTransform;
   }

}
