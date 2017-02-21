package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PelvisPoseErrorPacket;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.communication.subscribers.TimeStampedTransformBuffer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

@ContinuousIntegrationPlan(categories={IntegrationCategory.FAST})
public class NewPelvisPoseHistoryCorrectionTest
{
   private YoVariableRegistry registry;
   SimulationTestingParameters simulationTestingParameters = new SimulationTestingParameters();
   
   SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
   SimulationConstructionSet simulationConstructionSet;
   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final double estimatorDT = 0.001;
   
   private Robot robot;
   private ReferenceFrame pelvisReferenceFrame;
   private RigidBodyTransform pelvisTransformInWorldFrame = new RigidBodyTransform();
   private SixDoFJoint sixDofPelvisJoint;
   
   private final int pelvisBufferSize = 100;
   private final int numberOfTimeStamps = 50000;
   private final int bufferSize = numberOfTimeStamps;
   
   private ExternalPelvisPoseCreator externalPelvisPoseCreator;
   private NewPelvisPoseHistoryCorrection pelvisCorrector;
   private YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   
   int numberOfPelvisWaypoints = 11;
   TimeStampedTransformBuffer pelvisWaypointsTransformPoseBufferInWorldFrame = new TimeStampedTransformBuffer(numberOfPelvisWaypoints);

   int numberOfIcpOffsets = 16;
   TimeStampedTransformBuffer icpTransformPoseBufferInWorldFrame = new TimeStampedTransformBuffer(70);
   TimeStampedTransformBuffer icpOffsetsTransformPoseBuffer = new TimeStampedTransformBuffer(70);
   
   private boolean angleErrorTooBigDetectedAndPacketSent = false;
   
   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      setupRobot();
      registry = robot.getRobotsYoVariableRegistry();
      setupSim();
      setupCorrector();
      simulationConstructionSet.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      
   }

   @After
   public void showMemoryAfterTests()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   private void setupRobot()
   {
      robot = new Robot("dummy");

      pelvisReferenceFrame = new ReferenceFrame("pelvis", ReferenceFrame.getWorldFrame())
      {
         private static final long serialVersionUID = -6427490298776551499L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(pelvisTransformInWorldFrame);
         }
      };
      RigidBody rigidBody = new RigidBody("pelvis", pelvisReferenceFrame);
      sixDofPelvisJoint = new SixDoFJoint("pelvis", rigidBody, worldFrame);
   }

   private void setupSim()
   {
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(false);
      parameters.setDataBufferSize(bufferSize);
      
      simulationConstructionSet = new SimulationConstructionSet(robot, parameters);
      simulationConstructionSet.setDT(0.001, 1);
   }

   private void setupCorrector()
   {
      externalPelvisPoseCreator = new ExternalPelvisPoseCreator();
      pelvisCorrector = new NewPelvisPoseHistoryCorrection(sixDofPelvisJoint, estimatorDT, registry, pelvisBufferSize, yoGraphicsListRegistry, externalPelvisPoseCreator);
   }
   
   private void generatePelvisWayPoints()
   {
      RigidBodyTransform pelvisTransformInWorldFrame = new RigidBodyTransform();
      long timeStamp;
      Vector3D translation = new Vector3D();
      Quaternion rotation = new Quaternion();

      timeStamp = 0;
      translation.set(0.0, 0.0, 0.7);
      rotation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 5000;
      translation.set(1.0, 0.0, 0.5);
      rotation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 10000;
      translation.set(1.0, 1.0, 0.7);
      rotation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(5.0), Math.toRadians(5.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 15000;
      translation.set(1.0, 1.0, 0.6);
      rotation.setYawPitchRoll(Math.toRadians(-45.0), Math.toRadians(0.0), Math.toRadians(-1.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 20000;
      translation.set(3.0, -1.0, 0.6);
      rotation.setYawPitchRoll(Math.toRadians(-45.0), Math.toRadians(-12.0), Math.toRadians(0.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 25000;
      translation.set(3.0, -1.0, 0.6);
      rotation.setYawPitchRoll(Math.toRadians(135.0), Math.toRadians(0.0), Math.toRadians(0.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 30000;
      translation.set(1.0, 1.0, 0.9);
      rotation.setYawPitchRoll(Math.toRadians(135.0), Math.toRadians(0.0), Math.toRadians(-2.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 35000;
      translation.set(1.0, 1.0, 0.9);
      rotation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(-15.0), Math.toRadians(5.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 40000;
      translation.set(-1.0, 1.0, 0.5);
      rotation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(-1.0), Math.toRadians(-5.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 45000;
      translation.set(-1.0, -1.0, 0.7);
      rotation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 50000;
      translation.set(-1.0, -1.0, 0.2);
      rotation.setYawPitchRoll(Math.toRadians(180.0), Math.toRadians(10.0), Math.toRadians(10.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);
   }

   private void putPelvisWaypointInTransformBuffer(RigidBodyTransform pelvisTransformInWorldFrame, long timeStamp, Vector3D translation, Quaternion rotation)
   {
      pelvisTransformInWorldFrame.setTranslation(translation);
      pelvisTransformInWorldFrame.setRotation(rotation);
      pelvisWaypointsTransformPoseBufferInWorldFrame.put(pelvisTransformInWorldFrame, timeStamp);
   }

   private void generateSmallIcpOffsetsAroundPelvis()
   {
      Random random = new Random();

      for(long timeStamp = 3000; timeStamp <50000; timeStamp += 3000)
      {
         Vector3D translationOffset = RandomTools.generateRandomVector(random, 0.04);
         Quaternion rotationOffset = RandomTools.generateRandomQuaternion(random, 0.04);
         saveIcpOffsetInTransformBuffer(timeStamp, translationOffset, rotationOffset);
         generateIcpOffsetsWithRespectToPelvisInTransformBuffer(timeStamp, translationOffset, rotationOffset);
      }
   }
   
   private void saveIcpOffsetInTransformBuffer(long timeStamp, Vector3D translationOffset, Quaternion rotationOffset)
   {
      RigidBodyTransform icpOffsetTransform = new RigidBodyTransform(rotationOffset, translationOffset);
      icpOffsetsTransformPoseBuffer.put(icpOffsetTransform, timeStamp);
   }

   private void generateIcpOffsetsWithRespectToPelvisInTransformBuffer(long timeStamp, Vector3D translationOffset, Quaternion rotationOffset)
   {
      TimeStampedTransform3D pelvisTransformAtSpecificTimeStamp = new TimeStampedTransform3D();
      RigidBodyTransform pelvisTransformAtSpecificTimeStamp_Translation = new RigidBodyTransform();
      RigidBodyTransform pelvisTransformAtSpecificTimeStamp_Rotation = new RigidBodyTransform();
      
      RigidBodyTransform currentPelvisTransformModifiedWithOffset = new RigidBodyTransform();
      RigidBodyTransform smallOffsetsTransform_Translation = new RigidBodyTransform();
      RigidBodyTransform smallOffsetsTransform_Rotation = new RigidBodyTransform();
      
      pelvisWaypointsTransformPoseBufferInWorldFrame.findTransform(timeStamp, pelvisTransformAtSpecificTimeStamp);
      pelvisTransformAtSpecificTimeStamp_Translation.set(pelvisTransformAtSpecificTimeStamp.getTransform3D());
      pelvisTransformAtSpecificTimeStamp_Rotation.set(pelvisTransformAtSpecificTimeStamp_Translation);
      pelvisTransformAtSpecificTimeStamp_Translation.setRotationToZero();
      pelvisTransformAtSpecificTimeStamp_Rotation.setTranslationToZero();
      
      smallOffsetsTransform_Translation.setTranslationAndIdentityRotation(translationOffset);
      smallOffsetsTransform_Rotation.setRotationAndZeroTranslation(rotationOffset);
      
      currentPelvisTransformModifiedWithOffset.setIdentity();
      currentPelvisTransformModifiedWithOffset.multiply(pelvisTransformAtSpecificTimeStamp_Translation);
      currentPelvisTransformModifiedWithOffset.multiply(smallOffsetsTransform_Translation);
      currentPelvisTransformModifiedWithOffset.multiply(pelvisTransformAtSpecificTimeStamp_Rotation);
      currentPelvisTransformModifiedWithOffset.multiply(smallOffsetsTransform_Rotation);
      
      icpTransformPoseBufferInWorldFrame.put(currentPelvisTransformModifiedWithOffset, timeStamp);
   }

   
   private BooleanYoVariable isRotationCorrectionEnabled;
   private DoubleYoVariable maximumErrorTranslation;
   private DoubleYoVariable maximumErrorAngleInDegrees;
   
   @ContinuousIntegrationTest(estimatedDuration = 0.8)
   @Test(timeout = 30000)
   public void testTranslationCorrectionOnlyWithPelvisFollowingAKnownPathAndRandomLocalizationOffsets()
   {
      isRotationCorrectionEnabled = (BooleanYoVariable) registry.getVariable("ClippedSpeedOffsetErrorInterpolator", "isRotationCorrectionEnabled");
      isRotationCorrectionEnabled.set(false);
      maximumErrorTranslation = (DoubleYoVariable) registry.getVariable("ClippedSpeedOffsetErrorInterpolator", "maximumErrorTranslation");
      maximumErrorTranslation.set(Double.MAX_VALUE);
      maximumErrorAngleInDegrees = (DoubleYoVariable) registry.getVariable("ClippedSpeedOffsetErrorInterpolator", "maximumErrorAngleInDegrees");
      maximumErrorAngleInDegrees.set(Double.MAX_VALUE);
      
      generatePelvisWayPoints();
      generateSmallIcpOffsetsAroundPelvis();
      
      TimeStampedTransform3D pelvisTimeStampedTransform3D = new TimeStampedTransform3D();
      TimeStampedTransform3D icpTimeStampedTransform3D = new TimeStampedTransform3D();

      RigidBodyTransform pelvisBeforeCorrection_Translation = new RigidBodyTransform();
      RigidBodyTransform pelvisBeforeCorrection_Rotation = new RigidBodyTransform();
      RigidBodyTransform pelvisAfterCorrection = new RigidBodyTransform();
      RigidBodyTransform pelvisExpectedCorrection = new RigidBodyTransform();
      
      FramePose correctedPelvisverify = new FramePose(worldFrame); 
      YoFramePose correctedPelvisToVerifyTheTest = new YoFramePose("correctedPelvisToVerifyTheTest", worldFrame, registry);
      
      
      for (long timeStamp = 0; timeStamp < numberOfTimeStamps; timeStamp++)
      {
         pelvisWaypointsTransformPoseBufferInWorldFrame.findTransform(timeStamp, pelvisTimeStampedTransform3D);
         pelvisTransformInWorldFrame.set(pelvisTimeStampedTransform3D.getTransform3D());
         pelvisReferenceFrame.update();
         
         sixDofPelvisJoint.setPositionAndRotation(pelvisTimeStampedTransform3D.getTransform3D());
         sixDofPelvisJoint.updateFramesRecursively();
         pelvisBeforeCorrection_Translation.set(pelvisTimeStampedTransform3D.getTransform3D());
         pelvisBeforeCorrection_Translation.setRotationToZero();
         pelvisBeforeCorrection_Rotation.set(pelvisTimeStampedTransform3D.getTransform3D());
         pelvisBeforeCorrection_Rotation.setTranslationToZero();
         
         pelvisCorrector.doControl(timeStamp);
         pelvisAfterCorrection.set(sixDofPelvisJoint.getJointTransform3D());
         
         correctedPelvisverify.setPose(pelvisAfterCorrection);
         correctedPelvisToVerifyTheTest.set(correctedPelvisverify);
         
         if ( timeStamp > 3000 && ((timeStamp - 80) % 3000) == 0)
         {
            icpTransformPoseBufferInWorldFrame.findTransform(timeStamp - 80, icpTimeStampedTransform3D);
            StampedPosePacket newestStampedPosePacket = new StampedPosePacket("/pelvis", icpTimeStampedTransform3D, 1.0);
            externalPelvisPoseCreator.setNewestPose(newestStampedPosePacket);
         }
         
         simulationConstructionSet.tickAndUpdate();
         
         if((timeStamp-2000)%3000 == 0)
         {
            if(timeStamp >= icpOffsetsTransformPoseBuffer.getOldestTimestamp())
            {
               TimeStampedTransform3D temporaryTimeStampedTransform = new TimeStampedTransform3D();
               icpOffsetsTransformPoseBuffer.findTransform(timeStamp - 2000, temporaryTimeStampedTransform);
               
               RigidBodyTransform temporaryTransform = new RigidBodyTransform(temporaryTimeStampedTransform.getTransform3D());
               temporaryTransform.setRotationToZero(); //here we can do that because rotation correction is deactivated by default. This will need to be updated if we activate rotation correction
               
               pelvisExpectedCorrection.setIdentity();
               pelvisExpectedCorrection.multiply(pelvisBeforeCorrection_Translation);
               pelvisExpectedCorrection.multiply(temporaryTransform);
               pelvisExpectedCorrection.multiply(pelvisBeforeCorrection_Rotation);

               assertTrue(pelvisExpectedCorrection.epsilonEquals(pelvisAfterCorrection, 1e-4));
               
            }
         }
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 30000)
   public void testTooBigAngleErrorAreDetectedAndPacketIsSent()
   {
      boolean checkPacketHasBeenSentNextLoopIteration = false;
      generatePelvisWayPoints();
      generateIcpOffsetsAroundPelvisWithTooBigAngleErrors();
      
      TimeStampedTransform3D pelvisTimeStampedTransform3D = new TimeStampedTransform3D();
      TimeStampedTransform3D icpTimeStampedTransform3D = new TimeStampedTransform3D();
      
      RigidBodyTransform pelvisBeforeCorrection = new RigidBodyTransform();
      RigidBodyTransform pelvisAfterCorrection = new RigidBodyTransform();
      
      FramePose correctedPelvisverify = new FramePose(worldFrame); 
      YoFramePose correctedPelvisToVerifyTheTest = new YoFramePose("correctedPelvisToVerifyTheTest", worldFrame, registry);
      
      
      for (long timeStamp = 0; timeStamp < numberOfTimeStamps; timeStamp++)
      {
         pelvisWaypointsTransformPoseBufferInWorldFrame.findTransform(timeStamp, pelvisTimeStampedTransform3D);
         pelvisTransformInWorldFrame.set(pelvisTimeStampedTransform3D.getTransform3D());
         pelvisReferenceFrame.update();
         
         sixDofPelvisJoint.setPositionAndRotation(pelvisTimeStampedTransform3D.getTransform3D());
         sixDofPelvisJoint.updateFramesRecursively();
         pelvisBeforeCorrection.set(pelvisTimeStampedTransform3D.getTransform3D());
         
         pelvisCorrector.doControl(timeStamp);
         pelvisAfterCorrection.set(sixDofPelvisJoint.getJointTransform3D());
         if(checkPacketHasBeenSentNextLoopIteration)
         {
            assertTrue(angleErrorTooBigDetectedAndPacketSent);
            angleErrorTooBigDetectedAndPacketSent = false;
            checkPacketHasBeenSentNextLoopIteration = false;
         }
         
         if ( timeStamp > 3000 && ((timeStamp - 80) % 3000) == 0)
         {
            icpTransformPoseBufferInWorldFrame.findTransform(timeStamp - 80, icpTimeStampedTransform3D);
            StampedPosePacket newestStampedPosePacket = new StampedPosePacket("/pelvis", icpTimeStampedTransform3D, 1.0);
            externalPelvisPoseCreator.setNewestPose(newestStampedPosePacket);
            checkPacketHasBeenSentNextLoopIteration = true;
         }
         
         correctedPelvisverify.setPose(pelvisAfterCorrection);
         correctedPelvisToVerifyTheTest.set(correctedPelvisverify);
         
         simulationConstructionSet.tickAndUpdate();

         assertTrue(pelvisBeforeCorrection.epsilonEquals(pelvisAfterCorrection, 1e-4));

      }
   }

   private void generateIcpOffsetsAroundPelvisWithTooBigAngleErrors()
   {
      int index = 0;
      Random random = new Random();
      Quaternion[] orientationTooBigOffset = new Quaternion[16];
      Quaternion tempQuat = new Quaternion();
      tempQuat.setYawPitchRoll(Math.toRadians(10.1), Math.toRadians(0.0), Math.toRadians(0.0));
      orientationTooBigOffset[0] = tempQuat;
      tempQuat.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(10.5), Math.toRadians(0.0));
      orientationTooBigOffset[1] = tempQuat;
      tempQuat.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(11.0));
      orientationTooBigOffset[2] = tempQuat;
      tempQuat.setYawPitchRoll(Math.toRadians(10.5), Math.toRadians(22.0), Math.toRadians(0.0));
      orientationTooBigOffset[3] = tempQuat;
      tempQuat.setYawPitchRoll(Math.toRadians(30.0), Math.toRadians(0.0), Math.toRadians(15.0));
      orientationTooBigOffset[4] = tempQuat;
      tempQuat.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(15.0), Math.toRadians(20.0));
      orientationTooBigOffset[5] = tempQuat;
      tempQuat.setYawPitchRoll(Math.toRadians(10.15), Math.toRadians(20.0), Math.toRadians(30.0));
      orientationTooBigOffset[6] = tempQuat;
      tempQuat.setYawPitchRoll(Math.toRadians(-11.0), Math.toRadians(0.0), Math.toRadians(0.0));
      orientationTooBigOffset[7] = tempQuat;
      tempQuat.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(-12.0), Math.toRadians(0.0));
      orientationTooBigOffset[8] = tempQuat;
      tempQuat.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(-50.0));
      orientationTooBigOffset[9] = tempQuat;
      tempQuat.setYawPitchRoll(Math.toRadians(-22.0), Math.toRadians(-22.0), Math.toRadians(0.0));
      orientationTooBigOffset[10] = tempQuat;
      tempQuat.setYawPitchRoll(Math.toRadians(-30.0), Math.toRadians(0.0), Math.toRadians(-60.0));
      orientationTooBigOffset[11] = tempQuat;
      tempQuat.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(-12.0), Math.toRadians(-22.0));
      orientationTooBigOffset[12] = tempQuat;
      tempQuat.setYawPitchRoll(Math.toRadians(-12.0), Math.toRadians(-11.0), Math.toRadians(-10.2));
      orientationTooBigOffset[13] = tempQuat;
      tempQuat.setYawPitchRoll(Math.toRadians(12.0), Math.toRadians(-20.0), Math.toRadians(0.0));
      orientationTooBigOffset[14] = tempQuat;
      tempQuat.setYawPitchRoll(Math.toRadians(-20.0), Math.toRadians(0.0), Math.toRadians(0.9));
      orientationTooBigOffset[15] = tempQuat;
      
      for(long timeStamp = 3000; timeStamp <50000; timeStamp += 3000)
      {
         Vector3D translationOffset = RandomTools.generateRandomVector(random, 0.04);
         Quaternion rotationOffset = orientationTooBigOffset[index];
         saveIcpOffsetInTransformBuffer(timeStamp, translationOffset, rotationOffset);
         generateIcpOffsetsWithRespectToPelvisInTransformBuffer(timeStamp, translationOffset, rotationOffset);
         index++;
      }
   }

   /////////////////////////////////////////////////////////
   ///////   Necessary class to make the test work   ///////
   /////////////////////////////////////////////////////////
   private class ExternalPelvisPoseCreator implements PelvisPoseCorrectionCommunicatorInterface
   {
      private StampedPosePacket newestStampedPosePacket;
      boolean newPose;

      public void setNewestPose(StampedPosePacket newestStampedPosePacket)
      {
         this.newestStampedPosePacket = newestStampedPosePacket;
         newPose = true;
      }

      @Override
      public boolean hasNewPose()
      {
         return newPose;
      }

      @Override
      public StampedPosePacket getNewExternalPose()
      {
         newPose = false;
         return this.newestStampedPosePacket;
      }

      @Override
      public void receivedPacket(StampedPosePacket object)
      {
         //doNothing
      }

      @Override
      public void sendPelvisPoseErrorPacket(PelvisPoseErrorPacket pelvisPoseErrorPacket)
      {
         //doNothing
      }

      @Override
      public void sendLocalizationResetRequest(LocalizationPacket localizationPacket)
      {
         angleErrorTooBigDetectedAndPacketSent = true;
         
      }
   }
}
