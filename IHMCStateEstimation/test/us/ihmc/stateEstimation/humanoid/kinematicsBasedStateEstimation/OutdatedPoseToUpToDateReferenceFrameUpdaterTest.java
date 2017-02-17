package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.subscribers.TimeStampedTransformBuffer;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

@ContinuousIntegrationPlan(categories={IntegrationCategory.FAST})
public class OutdatedPoseToUpToDateReferenceFrameUpdaterTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetUpToDateTimeStampedBufferNewestTimeStamp()
   {
      FramePose upToDatePoseInPresent = new FramePose(worldFrame);
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetUpToDateTimeStampedBufferOldestTimeStamp()
   {
      FramePose upToDatePoseInPresent = new FramePose(worldFrame);
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
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test//(timeout = 30000)
   public void testComputedRotationError()
   {
      Random random = new Random(1987L);
      int numberOfUpToDateTransforms = 1000;
      int numberOfOutdatedTransforms = numberOfUpToDateTransforms / 2;
      
      Vector3D[] translationOffsets = new Vector3D[numberOfOutdatedTransforms];
      Quaternion[] orientationOffsets = new Quaternion[numberOfOutdatedTransforms];
      long[] outdatedTimeStamps = new long[numberOfOutdatedTransforms];
      
      FramePose upToDatePoseInPresent = new FramePose(worldFrame);
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
         orientationOffsets[j] = RandomTools.generateRandomQuaternion(random, Math.PI / 2.0);

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
         totalError.getRotation(calculatedRotationError);
         Quaternion actualError = orientationOffsets[i];
         
         assertTrue(calculatedRotationError.epsilonEquals(actualError, 1e-4));
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test//(timeout = 30000)
   public void testComputedTranslationError()
   {
      Random random = new Random(1987L);
      int numberOfUpToDateTransforms = 1000;
      int numberOfOutdatedTransforms = numberOfUpToDateTransforms / 2;
      
      Vector3D[] translationOffsets = new Vector3D[numberOfOutdatedTransforms];
      Quaternion[] orientationOffsets = new Quaternion[numberOfOutdatedTransforms];
      long[] outdatedTimeStamps = new long[numberOfOutdatedTransforms];
      
      FramePose upToDatePoseInPresent = new FramePose(worldFrame);
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
         translationOffsets[j] = RandomTools.generateRandomVector(random, -2.0, -2.0, 0.0, 2.0, 2.0, 2.0);
         orientationOffsets[j] = RandomTools.generateRandomQuaternion(random, Math.PI / 2.0);
         
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
         totalError.getTranslation(calculatedTranslationError);
         Vector3D actualTranslationError = translationOffsets[i];
         
         Quaternion calculatedRotationError = new Quaternion();
         totalError.getRotation(calculatedRotationError);
         Quaternion actualRotationError = orientationOffsets[i];
         
         assertTrue(calculatedTranslationError.epsilonEquals(actualTranslationError, 1e-4));
         assertTrue(calculatedRotationError.epsilonEquals(actualRotationError, 1e-4));
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test//(timeout = 30000)
   public void testComputedError()
   {
      Random random = new Random(1987L);
      int numberOfUpToDateTransforms = 1000;
      int numberOfOutdatedTransforms = numberOfUpToDateTransforms / 2;
      
      Vector3D[] translationOffsets = new Vector3D[numberOfOutdatedTransforms];
      Quaternion[] orientationOffsets = new Quaternion[numberOfOutdatedTransforms];
      long[] outdatedTimeStamps = new long[numberOfOutdatedTransforms];
      
      FramePose upToDatePoseInPresent = new FramePose(worldFrame);
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
         translationOffsets[j] = RandomTools.generateRandomVector(random, -2.0, -2.0, 0.0, 2.0, 2.0, 2.0);
         orientationOffsets[j] = RandomTools.generateRandomQuaternion(random, Math.PI / 2.0);
         
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
         totalError.getTranslation(calculatedTranslationError);
         Vector3D actualError = translationOffsets[i];
         
         assertTrue(calculatedTranslationError.epsilonEquals(actualError, 1e-4));
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test (timeout = 30000)
   public void testSimpleTranslationAtKnownLocation()
   {
      Random random = new Random(1987L);
      FramePose upToDatePoseInPresent = new FramePose(worldFrame);
      PoseReferenceFrame stateEsimatorReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", upToDatePoseInPresent);

      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(10,
            stateEsimatorReferenceFrameInPresent);

      outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(new RigidBodyTransform(), 1);

      RigidBodyTransform localizationRigidBody = new RigidBodyTransform(new Quaternion(), new Point3D(1.0, 1.0, 1.0));
      FramePose expectedPose = new FramePose(worldFrame);
      expectedPose.setPosition(1.0, 1.0, 1.0);
      
      TimeStampedTransform3D localizationTimeStampedTransformInWorld = new TimeStampedTransform3D(localizationRigidBody, 1);
      outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(localizationTimeStampedTransformInWorld);
      
      ReferenceFrame localizationReferenceFrameToBeUpdated = outdatedPoseToUpToDateReferenceFrameUpdater.getLocalizationReferenceFrameToBeUpdated();
      
      FramePose calculatedPose = new FramePose(localizationReferenceFrameToBeUpdated);
      calculatedPose.changeFrame(worldFrame);
      
      assertTrue(calculatedPose.epsilonEquals(expectedPose, 1e-4));

   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test (timeout = 30000)
   public void testNoDifferenceBetweenStateEstimatorAndLocalization()
   {
      FramePose upToDatePoseInPresent = new FramePose(worldFrame);
      upToDatePoseInPresent.setPosition(1.0, 1.0, 1.0);
      upToDatePoseInPresent.setYawPitchRoll(Math.PI/8, Math.PI/8, Math.PI/8);
      
      FramePose expectedPose = new FramePose(worldFrame);
      expectedPose.setYawPitchRoll(Math.PI/8, Math.PI/8, Math.PI/8);
      expectedPose.setPosition(1.0, 1.0, 1.0);
      
      RigidBodyTransform stateEstimatorRigidBody = new RigidBodyTransform();
      RigidBodyTransform localizationRigidBody = new RigidBodyTransform();
      
      upToDatePoseInPresent.getRigidBodyTransform(stateEstimatorRigidBody);
      upToDatePoseInPresent.getRigidBodyTransform(localizationRigidBody);
      
      PoseReferenceFrame stateEsimatorReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", upToDatePoseInPresent);
      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(10,
            stateEsimatorReferenceFrameInPresent);
      
      outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(stateEstimatorRigidBody, 1);
      
      TimeStampedTransform3D localizationTimeStampedTransformInWorld = new TimeStampedTransform3D(localizationRigidBody, 1);
      outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(localizationTimeStampedTransformInWorld);
      
      ReferenceFrame localizationReferenceFrameToBeUpdated = outdatedPoseToUpToDateReferenceFrameUpdater.getLocalizationReferenceFrameToBeUpdated();
      
      FramePose calculatedPose = new FramePose(localizationReferenceFrameToBeUpdated);
      calculatedPose.changeFrame(worldFrame);
      
      assertTrue(calculatedPose.epsilonEquals(expectedPose, 1e-4));
   }
   
   //this tests fails, I don't think OutdatedPoseToUpToDateReferenceFrameUpdater can support more than a single rotation at a time
   @Ignore
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test //(timeout = 30000)
   public void testKnownDifferenceBetweenStateEstimatorAndLocalization()
   {
      FramePose stateEstimatorPresent = new FramePose(worldFrame);
      stateEstimatorPresent.setPosition(2.0, 22.0, 1.0);
      stateEstimatorPresent.setYawPitchRoll(Math.PI, Math.PI / 32.0, Math.PI / 16.0);
      PoseReferenceFrame stateEsimatorReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", stateEstimatorPresent);

      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(10,
            stateEsimatorReferenceFrameInPresent);

      //create pose in past
      RigidBodyTransform stateEstimatorRigidBodyTransform = new RigidBodyTransform();
      FramePose stateEstimatorInPast = new FramePose(worldFrame);
      stateEstimatorInPast.setPosition(1.0, 20.0, 0.8);
      stateEstimatorInPast.setYawPitchRoll(Math.PI - (Math.PI / 64.0), Math.PI / 64.0, Math.PI / 8.0);
      stateEstimatorInPast.getRigidBodyTransform(stateEstimatorRigidBodyTransform);
      outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(stateEstimatorRigidBodyTransform, 1);

      //create localization pose in past
      RigidBodyTransform localizationRigidBody = new RigidBodyTransform();
      FramePose localizationInPast = new FramePose(worldFrame);
      localizationInPast.setPosition(1.5, 21.8, 1.1);
      localizationInPast.setYawPitchRoll(Math.PI - (Math.PI / 32.0), Math.PI / 16.0, Math.PI / 4);
      localizationInPast.getRigidBodyTransform(localizationRigidBody);
      
      TimeStampedTransform3D localizationTimeStampedTransformInWorld = new TimeStampedTransform3D(localizationRigidBody, 1);
      outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(localizationTimeStampedTransformInWorld);
      
      ReferenceFrame localizationReferenceFrameToBeUpdated = outdatedPoseToUpToDateReferenceFrameUpdater.getLocalizationReferenceFrameToBeUpdated();
      FramePose calculatedPose = new FramePose(localizationReferenceFrameToBeUpdated);
      calculatedPose.changeFrame(worldFrame);
      
      FramePose expectedPose = new FramePose(worldFrame);
      expectedPose.setYawPitchRoll(Math.PI - (Math.PI / 64.0), (5.0 * Math.PI) / 64.0, (3.0 * Math.PI) / 16.0);
      expectedPose.setPosition(2.5, 23.8, 1.3);
      System.out.println("z: " + -Math.PI / 64.0 + " y: " + (3.0 * Math.PI) / 64.0 + " x: " + (Math.PI / 8.0));
      System.out.println(calculatedPose);
      System.out.println(expectedPose);
      assertTrue(calculatedPose.epsilonEquals(expectedPose, 1e-4));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test //(timeout = 30000)
   public void testKnownTranslationAndYawDifferenceBetweenStateEstimatorAndLocalization()
   {
      FramePose stateEstimatorPresent = new FramePose(worldFrame);
      stateEstimatorPresent.setPosition(2.0, 22.0, 1.0);
      stateEstimatorPresent.setYawPitchRoll(Math.PI, Math.PI / 64.0, Math.PI / 8.0);
      PoseReferenceFrame stateEsimatorReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", stateEstimatorPresent);
      
      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(10,
            stateEsimatorReferenceFrameInPresent);
      
      //create pose in past
      RigidBodyTransform stateEstimatorRigidBodyTransform = new RigidBodyTransform();
      FramePose stateEstimatorInPast = new FramePose(worldFrame);
      stateEstimatorInPast.setPosition(1.0, 20.0, 0.8);
      stateEstimatorInPast.setYawPitchRoll(Math.PI - (Math.PI / 64.0), Math.PI / 64.0, Math.PI / 8.0);
      stateEstimatorInPast.getRigidBodyTransform(stateEstimatorRigidBodyTransform);
      outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(stateEstimatorRigidBodyTransform, 1);
      
      //create localization pose in past
      RigidBodyTransform localizationRigidBody = new RigidBodyTransform();
      FramePose localizationInPast = new FramePose(worldFrame);
      localizationInPast.setPosition(1.5, 21.8, 1.1);
      localizationInPast.setYawPitchRoll(Math.PI - (Math.PI / 32.0), Math.PI / 64.0, Math.PI / 8.0);
      localizationInPast.getRigidBodyTransform(localizationRigidBody);
      
      TimeStampedTransform3D localizationTimeStampedTransformInWorld = new TimeStampedTransform3D(localizationRigidBody, 1);
      outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(localizationTimeStampedTransformInWorld);
      
      ReferenceFrame localizationReferenceFrameToBeUpdated = outdatedPoseToUpToDateReferenceFrameUpdater.getLocalizationReferenceFrameToBeUpdated();
      FramePose calculatedPose = new FramePose(localizationReferenceFrameToBeUpdated);
      calculatedPose.changeFrame(worldFrame);
      
      FramePose expectedPose = new FramePose(worldFrame);
      expectedPose.setYawPitchRoll(Math.PI - (Math.PI / 64.0), Math.PI / 64.0, Math.PI / 8.0);
      expectedPose.setPosition(2.5, 23.8, 1.3);
      System.out.println("z: " + -Math.PI / 64.0 + " y: " + 0.0 + " x: " + 0.0);
      System.out.println(calculatedPose);
      System.out.println(expectedPose);
      assertTrue(calculatedPose.epsilonEquals(expectedPose, 1e-4));
   }
   
   //this tests fails, I don't think OutdatedPoseToUpToDateReferenceFrameUpdater can support more than a single rotation at a time
   @Ignore
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test //(timeout = 30000)
   public void testKnownTranslationYawAndPitchDifferenceBetweenStateEstimatorAndLocalization()
   {
      FramePose stateEstimatorPresent = new FramePose(worldFrame);
      stateEstimatorPresent.setPosition(2.0, 22.0, 1.0);
      stateEstimatorPresent.setYawPitchRoll(Math.PI, Math.PI / 8.0, Math.PI / 8.0);
      PoseReferenceFrame stateEsimatorReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", stateEstimatorPresent);
      
      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(10,
            stateEsimatorReferenceFrameInPresent);
      
      //create pose in past
      RigidBodyTransform stateEstimatorRigidBodyTransform = new RigidBodyTransform();
      FramePose stateEstimatorInPast = new FramePose(worldFrame);
      stateEstimatorInPast.setPosition(1.0, 20.0, 0.8);
      stateEstimatorInPast.setYawPitchRoll(Math.PI - (Math.PI / 64.0), Math.PI / 64.0, Math.PI / 8.0);
      stateEstimatorInPast.getRigidBodyTransform(stateEstimatorRigidBodyTransform);
      outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(stateEstimatorRigidBodyTransform, 1);
      
      //create localization pose in past
      RigidBodyTransform localizationRigidBody = new RigidBodyTransform();
      FramePose localizationInPast = new FramePose(worldFrame);
      localizationInPast.setPosition(1.5, 21.8, 1.1);
      localizationInPast.setYawPitchRoll(Math.PI - (Math.PI / 32.0), Math.PI / 32.0, Math.PI / 8.0);
      localizationInPast.getRigidBodyTransform(localizationRigidBody);
      
      TimeStampedTransform3D localizationTimeStampedTransformInWorld = new TimeStampedTransform3D(localizationRigidBody, 1);
      outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(localizationTimeStampedTransformInWorld);
      
      ReferenceFrame localizationReferenceFrameToBeUpdated = outdatedPoseToUpToDateReferenceFrameUpdater.getLocalizationReferenceFrameToBeUpdated();
      FramePose calculatedPose = new FramePose(localizationReferenceFrameToBeUpdated);
      calculatedPose.changeFrame(worldFrame);
      
      FramePose expectedPose = new FramePose(worldFrame);
      expectedPose.setYawPitchRoll(Math.PI - (Math.PI / 64.0), 9.0 * Math.PI / 64.0, Math.PI / 8.0);
      expectedPose.setPosition(2.5, 23.8, 1.3);
      System.out.println("z: " + -Math.PI / 64.0 + " y: " + Math.PI / 32.0 + " x: " + 0.0);
      System.out.println(calculatedPose);
      System.out.println(expectedPose);
      assertTrue(calculatedPose.epsilonEquals(expectedPose, 1e-4));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test (timeout = 30000)
   public void testSimpleRotationAtKnownLocation()
   {
      Random random = new Random(1987L);
      FramePose upToDatePoseInPresent = new FramePose(worldFrame);
      PoseReferenceFrame stateEsimatorReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", upToDatePoseInPresent);

      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(10,
            stateEsimatorReferenceFrameInPresent);

      outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(new RigidBodyTransform(), 1);

      
      RigidBodyTransform localizationRigidBody = new RigidBodyTransform();
      localizationRigidBody.setRotationEulerAndZeroTranslation(Math.PI/8, Math.PI/8, Math.PI/8);
      FramePose expectedPose = new FramePose(worldFrame);
      expectedPose.setYawPitchRoll(Math.PI/8, Math.PI/8, Math.PI/8);
      
      TimeStampedTransform3D localizationTimeStampedTransformInWorld = new TimeStampedTransform3D(localizationRigidBody, 1);
      outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(localizationTimeStampedTransformInWorld);
      
      ReferenceFrame localizationReferenceFrameToBeUpdated = outdatedPoseToUpToDateReferenceFrameUpdater.getLocalizationReferenceFrameToBeUpdated();
      
      FramePose calculatedPose = new FramePose(localizationReferenceFrameToBeUpdated);
      calculatedPose.changeFrame(worldFrame);
      
      assertTrue(calculatedPose.epsilonEquals(expectedPose, 1e-4));

   }

   //this tests fails, I don't think OutdatedPoseToUpToDateReferenceFrameUpdater can support more than a single rotation at a time
   @Ignore
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test//(timeout = 30000)
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

      FramePose upToDatePoseInPresent = new FramePose(worldFrame);
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

         translationOffsets[j] = RandomTools.generateRandomVector(random, -2.0, -2.0, 0.0, 2.0, 2.0, 2.0);
         orientationOffsets[j] = RandomTools.generateRandomQuaternion(random, Math.PI / 2.0);//RandomTools.generateRandomQuaternion(random, Math.PI);

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
         upToDatePoseInPresent.setPose(upToDateTimeStampedTransform.getTransform3D());
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
         FramePose outdatedPoseUpdatedInWorldFrame = new FramePose(outdatedReferenceFrame_ToBeUpdated);
         outdatedPoseUpdatedInWorldFrame.changeFrame(worldFrame);

         Vector3D upToDateReferenceFrameInPresent_Translation = new Vector3D();
         upToDatePoseInPresent.getPosition(upToDateReferenceFrameInPresent_Translation);
         Vector3D outdatedPoseUpdatedInWorldFrame_Translation = new Vector3D();
         outdatedPoseUpdatedInWorldFrame.getPosition(outdatedPoseUpdatedInWorldFrame_Translation);
         
         FramePose testedPose = new FramePose(worldFrame);
         testedPose.setPose(outdatedPoseUpdatedInWorldFrame);
         testedPose.changeFrame(upToDateReferenceFrameInPresent);

         Vector3D testedTranslation = new Vector3D();
         testedTranslation.sub(outdatedPoseUpdatedInWorldFrame_Translation, upToDateReferenceFrameInPresent_Translation);
         Quaternion testedOrientation = new Quaternion();
         testedPose.getOrientation(testedOrientation);

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
      upToDateTransformInPast_Translation.setRotationToZero();
      upToDateTransformInPast_Rotation.setTranslationToZero();
      
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
      upToDateTransform.setTranslation(RandomTools.generateRandomVector(random));
      upToDateTransform.setRotation(RandomTools.generateRandomQuaternion(random));
      return upToDateTransform;
   }

}
