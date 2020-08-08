package us.ihmc.avatar.stateEstimationEndToEndTests;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.LocalizationPacket;
import controller_msgs.msg.dds.PelvisPoseErrorPacket;
import controller_msgs.msg.dds.StampedPosePacket;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.PelvisPoseHistoryCorrection;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class PelvisPoseHistoryCorrectionUsingSimpleRobotTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

//   private static final boolean showGUI = false;
   //TODO get that from the StateEstimatorParameters
   private static final boolean USE_ROTATION_CORRECTION = false;
   
   private YoRegistry registry;
   private RectangleRobot robot;
   private SimulationConstructionSet simulationConstructionSet;
   private RigidBodyTransform robotPose = new RigidBodyTransform();

   private YoDouble confidenceFactor; // target for alpha filter
   private YoDouble interpolationTranslationAlphaFilterBreakFrequency;
   private YoDouble interpolationTranslationAlphaFilterAlphaValue;
   private YoDouble interpolationTranslationAlphaFilter;
   private YoDouble interpolationRotationAlphaFilterBreakFrequency;
   private YoDouble interpolationRotationAlphaFilterAlphaValue;
   private YoDouble interpolationRotationAlphaFilter;
   private YoLong seNonProcessedPelvisTimeStamp;
   private YoDouble maxTranslationVelocityClip;
   private YoDouble maxRotationVelocityClip;
   private YoDouble translationClippedAlphaValue;
   private YoDouble rotationClippedAlphaValue;

   private YoDouble nonCorrectedPelvis_x;
   private YoDouble nonCorrectedPelvis_y;
   private YoDouble nonCorrectedPelvis_z;
   private YoDouble nonCorrectedPelvis_yaw;
   private YoDouble nonCorrectedPelvis_pitch;
   private YoDouble nonCorrectedPelvis_roll;
   private YoDouble correctedPelvis_x;
   private YoDouble correctedPelvis_y;
   private YoDouble correctedPelvis_z;
   private YoDouble correctedPelvis_yaw;
   private YoDouble correctedPelvis_pitch;
   private YoDouble correctedPelvis_roll;
   private YoDouble seBackInTimeFrame_x;
   private YoDouble seBackInTimeFrame_y;
   private YoDouble seBackInTimeFrame_z;
   private YoDouble seBackInTimeFrame_yaw;
   private YoDouble seBackInTimeFrame_pitch;
   private YoDouble seBackInTimeFrame_roll;
   private YoDouble localizationBackInTimeFrame_x;
   private YoDouble localizationBackInTimeFrame_y;
   private YoDouble localizationBackInTimeFrame_z;
   private YoDouble localizationBackInTimeFrame_yaw;
   private YoDouble localizationBackInTimeFrame_pitch;
   private YoDouble localizationBackInTimeFrame_roll;
   
   private YoDouble totalTranslationErrorFrame_x;
   private YoDouble totalTranslationErrorFrame_y;
   private YoDouble totalTranslationErrorFrame_z;
   private YoDouble totalTranslationErrorFrame_yaw;
   private YoDouble totalTranslationErrorFrame_pitch;
   private YoDouble totalTranslationErrorFrame_roll;
   
   private YoDouble totalRotationErrorFrame_x;
   private YoDouble totalRotationErrorFrame_y;
   private YoDouble totalRotationErrorFrame_z;
   private YoDouble totalRotationErrorFrame_yaw;
   private YoDouble totalRotationErrorFrame_pitch;
   private YoDouble totalRotationErrorFrame_roll;
   
   private YoDouble interpolatedTranslationCorrectionFrame_x;
   private YoDouble interpolatedTranslationCorrectionFrame_y;
   private YoDouble interpolatedTranslationCorrectionFrame_z;
   private YoDouble interpolatedTranslationCorrectionFrame_yaw;
   private YoDouble interpolatedTranslationCorrectionFrame_pitch;
   private YoDouble interpolatedTranslationCorrectionFrame_roll;
   
   private YoDouble interpolatedRotationCorrectionFrame_x;
   private YoDouble interpolatedRotationCorrectionFrame_y;
   private YoDouble interpolatedRotationCorrectionFrame_z;
   private YoDouble interpolatedRotationCorrectionFrame_yaw;
   private YoDouble interpolatedRotationCorrectionFrame_pitch;
   private YoDouble interpolatedRotationCorrectionFrame_roll;
   
   private YoDouble interpolationTranslationStartFrame_x;
   private YoDouble interpolationTranslationStartFrame_y;
   private YoDouble interpolationTranslationStartFrame_z;
   private YoDouble interpolationTranslationStartFrame_yaw;
   private YoDouble interpolationTranslationStartFrame_pitch;
   private YoDouble interpolationTranslationStartFrame_roll;
   
   private YoDouble interpolationRotationStartFrame_x;
   private YoDouble interpolationRotationStartFrame_y;
   private YoDouble interpolationRotationStartFrame_z;
   private YoDouble interpolationRotationStartFrame_yaw;
   private YoDouble interpolationRotationStartFrame_pitch;
   private YoDouble interpolationRotationStartFrame_roll;

   private BlockingSimulationRunner bsr;
   private SixDoFJoint sixDofPelvisJoint;
   private ExternalPelvisPoseCreator externalPelvisPoseCreator;
   private FloatingJoint floatingJoint;
   private ReferenceFrame refFrame;

   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   public static void main(String[] args)
   {
      String[] vars = new String[] { "nonCorrectedPelvis", "correctedPelvis", "seBackInTimeFrame", "localizationBackInTimeFrame", "totalErrorFrame",
            "interpolatedCorrectionFrame", "interpolationRotationStartFrame" };
      String[] postFix = new String[] { "_x", "_y", "_z", "_yaw", "_pitch", "_roll" };

      for (int i = 0; i < vars.length; i++)
      {
         for (int j = 0; j < postFix.length; j++)
         {
            System.out.println("private YoDouble " + vars[i] + postFix[j] + ";");
         }
      }
      for (int i = 0; i < vars.length; i++)
      {
         for (int j = 0; j < postFix.length; j++)
         {
            System.out.println(vars[i] + postFix[j] + " = (YoDouble) registry.getVariable(namespace, \"" + vars[i] + postFix[j] + "\"); ");
         }
      }
   }

   private void setupYoVariables(YoRegistry registry, String namespace)
   {
      interpolationTranslationAlphaFilter = (YoDouble) registry.findVariable(namespace, "PelvisTranslationErrorCorrectionAlphaFilter");
      interpolationTranslationAlphaFilterAlphaValue = (YoDouble) registry.findVariable(namespace, "interpolationTranslationAlphaFilterAlphaValue");
      interpolationTranslationAlphaFilterBreakFrequency = (YoDouble) registry.findVariable(namespace, "interpolationTranslationAlphaFilterBreakFrequency");
      interpolationRotationAlphaFilter = (YoDouble) registry.findVariable(namespace, "PelvisRotationErrorCorrectionAlphaFilter");
      interpolationRotationAlphaFilterAlphaValue = (YoDouble) registry.findVariable(namespace, "interpolationRotationAlphaFilterAlphaValue");
      interpolationRotationAlphaFilterBreakFrequency = (YoDouble) registry.findVariable(namespace, "interpolationRotationAlphaFilterBreakFrequency");
      confidenceFactor = (YoDouble) registry.findVariable(namespace, "PelvisErrorCorrectionConfidenceFactor");
      seNonProcessedPelvisTimeStamp = (YoLong) registry.findVariable(namespace, "seNonProcessedPelvis_timestamp");
      maxTranslationVelocityClip = (YoDouble) registry.findVariable(namespace, "maxTranslationVelocityClip");
      maxRotationVelocityClip = (YoDouble) registry.findVariable(namespace, "maxRotationVelocityClip");
      translationClippedAlphaValue = (YoDouble) registry.findVariable(namespace, "translationClippedAlphaValue");
      rotationClippedAlphaValue = (YoDouble) registry.findVariable(namespace, "rotationClippedAlphaValue");

      nonCorrectedPelvis_x = (YoDouble) registry.findVariable(namespace, "nonCorrectedPelvis_x");
      nonCorrectedPelvis_y = (YoDouble) registry.findVariable(namespace, "nonCorrectedPelvis_y");
      nonCorrectedPelvis_z = (YoDouble) registry.findVariable(namespace, "nonCorrectedPelvis_z");
      nonCorrectedPelvis_yaw = (YoDouble) registry.findVariable(namespace, "nonCorrectedPelvis_yaw");
      nonCorrectedPelvis_pitch = (YoDouble) registry.findVariable(namespace, "nonCorrectedPelvis_pitch");
      nonCorrectedPelvis_roll = (YoDouble) registry.findVariable(namespace, "nonCorrectedPelvis_roll");
      correctedPelvis_x = (YoDouble) registry.findVariable(namespace, "correctedPelvis_x");
      correctedPelvis_y = (YoDouble) registry.findVariable(namespace, "correctedPelvis_y");
      correctedPelvis_z = (YoDouble) registry.findVariable(namespace, "correctedPelvis_z");
      correctedPelvis_yaw = (YoDouble) registry.findVariable(namespace, "correctedPelvis_yaw");
      correctedPelvis_pitch = (YoDouble) registry.findVariable(namespace, "correctedPelvis_pitch");
      correctedPelvis_roll = (YoDouble) registry.findVariable(namespace, "correctedPelvis_roll");
      seBackInTimeFrame_x = (YoDouble) registry.findVariable(namespace, "seBackInTimeFrame_x");
      seBackInTimeFrame_y = (YoDouble) registry.findVariable(namespace, "seBackInTimeFrame_y");
      seBackInTimeFrame_z = (YoDouble) registry.findVariable(namespace, "seBackInTimeFrame_z");
      seBackInTimeFrame_yaw = (YoDouble) registry.findVariable(namespace, "seBackInTimeFrame_yaw");
      seBackInTimeFrame_pitch = (YoDouble) registry.findVariable(namespace, "seBackInTimeFrame_pitch");
      seBackInTimeFrame_roll = (YoDouble) registry.findVariable(namespace, "seBackInTimeFrame_roll");
      localizationBackInTimeFrame_x = (YoDouble) registry.findVariable(namespace, "localizationBackInTimeFrame_x");
      localizationBackInTimeFrame_y = (YoDouble) registry.findVariable(namespace, "localizationBackInTimeFrame_y");
      localizationBackInTimeFrame_z = (YoDouble) registry.findVariable(namespace, "localizationBackInTimeFrame_z");
      localizationBackInTimeFrame_yaw = (YoDouble) registry.findVariable(namespace, "localizationBackInTimeFrame_yaw");
      localizationBackInTimeFrame_pitch = (YoDouble) registry.findVariable(namespace, "localizationBackInTimeFrame_pitch");
      localizationBackInTimeFrame_roll = (YoDouble) registry.findVariable(namespace, "localizationBackInTimeFrame_roll");
      
      totalTranslationErrorFrame_x = (YoDouble) registry.findVariable(namespace, "totalTranslationErrorFrame_x");
      totalTranslationErrorFrame_y = (YoDouble) registry.findVariable(namespace, "totalTranslationErrorFrame_y");
      totalTranslationErrorFrame_z = (YoDouble) registry.findVariable(namespace, "totalTranslationErrorFrame_z");
      totalTranslationErrorFrame_yaw = (YoDouble) registry.findVariable(namespace, "totalTranslationErrorFrame_yaw");
      totalTranslationErrorFrame_pitch = (YoDouble) registry.findVariable(namespace, "totalTranslationErrorFrame_pitch");
      totalTranslationErrorFrame_roll = (YoDouble) registry.findVariable(namespace, "totalTranslationErrorFrame_roll");
      
      totalRotationErrorFrame_x = (YoDouble) registry.findVariable(namespace, "totalRotationErrorFrame_x");
      totalRotationErrorFrame_y = (YoDouble) registry.findVariable(namespace, "totalRotationErrorFrame_y");
      totalRotationErrorFrame_z = (YoDouble) registry.findVariable(namespace, "totalRotationErrorFrame_z");
      totalRotationErrorFrame_yaw = (YoDouble) registry.findVariable(namespace, "totalRotationErrorFrame_yaw");
      totalRotationErrorFrame_pitch = (YoDouble) registry.findVariable(namespace, "totalRotationErrorFrame_pitch");
      totalRotationErrorFrame_roll = (YoDouble) registry.findVariable(namespace, "totalRotationErrorFrame_roll");
      
      interpolatedTranslationCorrectionFrame_x = (YoDouble) registry.findVariable(namespace, "interpolatedTranslationCorrectionFrame_x");
      interpolatedTranslationCorrectionFrame_y = (YoDouble) registry.findVariable(namespace, "interpolatedTranslationCorrectionFrame_y");
      interpolatedTranslationCorrectionFrame_z = (YoDouble) registry.findVariable(namespace, "interpolatedTranslationCorrectionFrame_z");
      interpolatedTranslationCorrectionFrame_yaw = (YoDouble) registry.findVariable(namespace, "interpolatedTranslationCorrectionFrame_yaw");
      interpolatedTranslationCorrectionFrame_pitch = (YoDouble) registry.findVariable(namespace, "interpolatedTranslationCorrectionFrame_pitch");
      interpolatedTranslationCorrectionFrame_roll = (YoDouble) registry.findVariable(namespace, "interpolatedTranslationCorrectionFrame_roll");
      
      interpolatedRotationCorrectionFrame_x = (YoDouble) registry.findVariable(namespace, "interpolatedRotationCorrectionFrame_x");
      interpolatedRotationCorrectionFrame_y = (YoDouble) registry.findVariable(namespace, "interpolatedRotationCorrectionFrame_y");
      interpolatedRotationCorrectionFrame_z = (YoDouble) registry.findVariable(namespace, "interpolatedRotationCorrectionFrame_z");
      interpolatedRotationCorrectionFrame_yaw = (YoDouble) registry.findVariable(namespace, "interpolatedRotationCorrectionFrame_yaw");
      interpolatedRotationCorrectionFrame_pitch = (YoDouble) registry.findVariable(namespace, "interpolatedRotationCorrectionFrame_pitch");
      interpolatedRotationCorrectionFrame_roll = (YoDouble) registry.findVariable(namespace, "interpolatedRotationCorrectionFrame_roll");
      
      interpolationTranslationStartFrame_x = (YoDouble) registry.findVariable(namespace, "interpolationTranslationStartFrame_x");
      interpolationTranslationStartFrame_y = (YoDouble) registry.findVariable(namespace, "interpolationTranslationStartFrame_y");
      interpolationTranslationStartFrame_z = (YoDouble) registry.findVariable(namespace, "interpolationTranslationStartFrame_z");
      interpolationTranslationStartFrame_yaw = (YoDouble) registry.findVariable(namespace, "interpolationTranslationStartFrame_yaw");
      interpolationTranslationStartFrame_pitch = (YoDouble) registry.findVariable(namespace, "interpolationTranslationStartFrame_pitch");
      interpolationTranslationStartFrame_roll = (YoDouble) registry.findVariable(namespace, "interpolationTranslationStartFrame_roll");
      
      interpolationRotationStartFrame_x = (YoDouble) registry.findVariable(namespace, "interpolationRotationStartFrame_x");
      interpolationRotationStartFrame_y = (YoDouble) registry.findVariable(namespace, "interpolationRotationStartFrame_y");
      interpolationRotationStartFrame_z = (YoDouble) registry.findVariable(namespace, "interpolationRotationStartFrame_z");
      interpolationRotationStartFrame_yaw = (YoDouble) registry.findVariable(namespace, "interpolationRotationStartFrame_yaw");
      interpolationRotationStartFrame_pitch = (YoDouble) registry.findVariable(namespace, "interpolationRotationStartFrame_pitch");
      interpolationRotationStartFrame_roll = (YoDouble) registry.findVariable(namespace, "interpolationRotationStartFrame_roll");

   }

   private void setupRobot()
   {
      robot = new RectangleRobot();
      robot.setDynamic(false);
      robot.setGravity(0);
      registry = robot.getRobotsYoRegistry();
      

      floatingJoint = (FloatingJoint) robot.getRootJoints().get(0);
      refFrame = new ReferenceFrame("pelvis", ReferenceFrame.getWorldFrame(), true, false)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(robotPose);
         }
      };
      RigidBodyBasics rigidBody = new RigidBody("pelvis", refFrame);
      sixDofPelvisJoint = new SixDoFJoint("pelvis", rigidBody);
   }

   private void setupSim()
   {
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(simulationTestingParameters.getCreateGUI());
      
      simulationConstructionSet = new SimulationConstructionSet(robot, parameters);
      simulationConstructionSet.setDT(0.001, 1);
      bsr = new BlockingSimulationRunner(simulationConstructionSet, 60.0 * 10.0);
      Thread simThread = new Thread(simulationConstructionSet);
      simThread.start();
   }

   private void setupCorrector()
   {
      externalPelvisPoseCreator = new ExternalPelvisPoseCreator();
      PelvisPoseHistoryCorrection pelvisCorrector = new PelvisPoseHistoryCorrection(sixDofPelvisJoint, Conversions.nanosecondsToSeconds(1000000), registry, 100,
            externalPelvisPoseCreator);
      PelvisPoseHistoryCorrectorController robotController = new PelvisPoseHistoryCorrectorController(pelvisCorrector, simulationConstructionSet);
      robot.setController(robotController);
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 0.015 , 0.015);
   }

	@Test
   public void testRandomInterpolationFinals() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      Random random = new Random(1984L);

      setupRobot();
      setupSim();
      setupCorrector();
      setupYoVariables(registry, "PelvisPoseHistoryCorrection");
      if (USE_ROTATION_CORRECTION)
      {
         assertTrue(testInterpolationForTranslationAndRotationToRandomTargetsFromOrigin(random, externalPelvisPoseCreator, registry, 10));
         assertTrue(testInterpolationForTranslationAndRotationToRandomTargetsFromSpecificLocations(random, externalPelvisPoseCreator, registry, 10));
         assertTrue(testYawForTranslationAndRotation(random, externalPelvisPoseCreator, registry, 10));
      }
      else
      {
         assertTrue(testInterpolationForTranslationToRandomTargetsFromOrigin(random, externalPelvisPoseCreator, registry, 10));
         assertTrue(testInterpolationForTranslationToRandomTargetsFromSpecificLocations(random, externalPelvisPoseCreator, registry, 10));
         assertTrue(testYawForTranslation(random, externalPelvisPoseCreator, registry, 10));
      }
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   private RigidBodyTransform[] createRandomCorrectionTargets(Random random, int numOfTargets)
   {
      RigidBodyTransform[] targets = new RigidBodyTransform[numOfTargets];
      for (int i = 0; i < 10; i++)
      {
         targets[i] = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      }
      return targets;
   }

   private RigidBodyTransform[] createYawOnlyCorrectionTargets(Random random, int numTargets)
   {
      RigidBodyTransform[] targets = new RigidBodyTransform[numTargets];
      Quaternion rot = new Quaternion();
      for (int i = 0; i < 10; i++)
      {
         targets[i] = new RigidBodyTransform();
         rot.setYawPitchRoll((random.nextDouble() * Math.PI * 2) - Math.PI, 0, 0);
         targets[i].getRotation().set(rot);
      }
      return targets;

   }

   private boolean testInterpolationForTranslationAndRotationToRandomTargetsFromOrigin(Random random, ExternalPelvisPoseCreator externalPelvisPoseCreator, YoRegistry registry, int numTargets)
         throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 5.0, 5.0);
      setPelvisPoseHistoryCorrectorMaxVelocity(registry, 100.0, 100.0);

      RigidBodyTransform[] targets = createRandomCorrectionTargets(random, numTargets);
      boolean success = true;

      Vector4D error = new Vector4D();
      Vector3D targetTranslation = new Vector3D();
      RotationMatrix targetRotation = new RotationMatrix();
      double targetYaw = 0;
      double translationFudgeFactor = 0.01;
      double rotationFudgeFactor = 0.01;
      double largestError = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < targets.length; i++)
      {
         targetTranslation.set(targets[i].getTranslation());
         targetRotation.set(targets[i].getRotation());
         targetYaw = targetRotation.getYaw();
         error.set(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ(), targetYaw);

         long timeStamp = Conversions.secondsToNanoseconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = HumanoidMessageTools.createStampedPosePacket("/pelvis", timeStampedTransform, 1.0);

         success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 3);
         externalPelvisPoseCreator.setNewestPose(posePacket);

         while (translationClippedAlphaValue.getDoubleValue() > 0.2 || rotationClippedAlphaValue.getDoubleValue() > 0.2)
         {
            success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 3);
         }

         while (translationClippedAlphaValue.getDoubleValue() < 0.9999999 && rotationClippedAlphaValue.getDoubleValue() <0.9999999)
         {
            success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 100);
         }

         double xError = Math.abs(correctedPelvis_x.getDoubleValue() - error.getX());
         double yError = Math.abs(correctedPelvis_y.getDoubleValue() - error.getY());
         double zError = Math.abs(correctedPelvis_z.getDoubleValue() - error.getZ());
         double yawError = Math.abs(correctedPelvis_yaw.getDoubleValue() - error.getS());

         if (xError > largestError)
            largestError = xError;
         if (yError > largestError)
            largestError = yError;
         if (zError > largestError)
            largestError = zError;
         if (yawError > largestError)
            largestError = yawError;

         success &= xError <= translationFudgeFactor;
         success &= yError <= translationFudgeFactor;
         success &= zError <= translationFudgeFactor;
         success &= yawError <= rotationFudgeFactor;

         error.set(xError, yError, zError, yawError);
      }
      System.out.println(" max fudge factor: " + largestError);
      return success;
   }
   
   private boolean testInterpolationForTranslationToRandomTargetsFromOrigin(Random random, ExternalPelvisPoseCreator externalPelvisPoseCreator, YoRegistry registry, int numTargets)
         throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 5.0, 5.0);
      setPelvisPoseHistoryCorrectorMaxVelocity(registry, 100.0, 100.0);
      
      RigidBodyTransform[] targets = createRandomCorrectionTargets(random, numTargets);
      boolean success = true;
      
      Vector4D error = new Vector4D();
      Vector3D targetTranslation = new Vector3D();
      RotationMatrix targetRotation = new RotationMatrix();
      double targetYaw = 0;
      double translationFudgeFactor = 0.01;
      double rotationFudgeFactor = 0.035;
      double largestError = Double.NEGATIVE_INFINITY;
      
      for (int i = 0; i < targets.length; i++)
      {
         targetTranslation.set(targets[i].getTranslation());
         targetRotation.set(targets[i].getRotation());
         targetYaw = targetRotation.getYaw();
         error.set(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ(), targetYaw);
         
         long timeStamp = Conversions.secondsToNanoseconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = HumanoidMessageTools.createStampedPosePacket("/pelvis", timeStampedTransform, 1.0);
         
         success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 3);
         externalPelvisPoseCreator.setNewestPose(posePacket);

         while (translationClippedAlphaValue.getDoubleValue() > 0.2)
         {
            success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 3);
         }
         
         while (translationClippedAlphaValue.getDoubleValue() < 0.9999999)
         {
            success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 100);
         }
         
         double xError = Math.abs(correctedPelvis_x.getDoubleValue() - error.getX());
         double yError = Math.abs(correctedPelvis_y.getDoubleValue() - error.getY());
         double zError = Math.abs(correctedPelvis_z.getDoubleValue() - error.getZ());
         double yawError = Math.abs(correctedPelvis_yaw.getDoubleValue() - error.getS());
         
         if (xError > largestError)
            largestError = xError;
         if (yError > largestError)
            largestError = yError;
         if (zError > largestError)
            largestError = zError;
         
         success &= xError <= translationFudgeFactor;
         success &= yError <= translationFudgeFactor;
         success &= zError <= translationFudgeFactor;
         success &= MathTools.epsilonEquals(Math.abs(targetYaw), yawError, rotationFudgeFactor); // here, rotation error are not exectued, so the error should be the same as the target
         
         error.set(xError, yError, zError, yawError);
      }
      System.out.println(" max fudge factor: " + largestError);
      return success;
   }

   private boolean testInterpolationForTranslationAndRotationToRandomTargetsFromSpecificLocations(Random random, ExternalPelvisPoseCreator externalPelvisPoseCreator, YoRegistry registry,
         int numTargets) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 5.0, 5.0);
      setPelvisPoseHistoryCorrectorMaxVelocity(registry, 100.0, 100.0);

      RigidBodyTransform[] targets = createRandomCorrectionTargets(random, numTargets);
      boolean success = true;

      Vector4D error = new Vector4D();
      Vector3D targetTranslation = new Vector3D();
      RotationMatrix targetRotation = new RotationMatrix();
      double targetYaw = 0;
      double translationFudgeFactor = 0.01;
      double rotationFudgeFactor = 0.01;
      double largestError = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < targets.length; i++)
      {
         targetTranslation.set(targets[i].getTranslation());
         targetRotation.set(targets[i].getRotation());
         targetYaw = targetRotation.getYaw();
         error.set(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ(), targetYaw);

         robotPose.getTranslation().set((double) i, (double) i, (double) i);

         long timeStamp = Conversions.secondsToNanoseconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = HumanoidMessageTools.createStampedPosePacket("/pelvis", timeStampedTransform, 1.0);

         success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 3);
         externalPelvisPoseCreator.setNewestPose(posePacket);

         while (translationClippedAlphaValue.getDoubleValue() > 0.2 || rotationClippedAlphaValue.getDoubleValue() > 0.2)
         {
            success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 3);
         }

         while (translationClippedAlphaValue.getDoubleValue() < 0.9999999 && rotationClippedAlphaValue.getDoubleValue() < 0.9999999)
         {
            success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 100);
         }

         double xError = Math.abs(correctedPelvis_x.getDoubleValue() - error.getX());
         double yError = Math.abs(correctedPelvis_y.getDoubleValue() - error.getY());
         double zError = Math.abs(correctedPelvis_z.getDoubleValue() - error.getZ());
         double yawError = Math.abs(correctedPelvis_yaw.getDoubleValue() - error.getS());

         if (xError > largestError)
            largestError = xError;
         if (yError > largestError)
            largestError = yError;
         if (zError > largestError)
            largestError = zError;
         if (yawError > largestError)
            largestError = yawError;

         success &= xError <= translationFudgeFactor;
         success &= yError <= translationFudgeFactor;
         success &= zError <= translationFudgeFactor;
         success &= yawError <= rotationFudgeFactor;

         error.set(xError, yError, zError, yawError);
      }
      System.out.println(" max fudge factor: " + largestError);
      return success;
   }
   
   private boolean testInterpolationForTranslationToRandomTargetsFromSpecificLocations(Random random, ExternalPelvisPoseCreator externalPelvisPoseCreator, YoRegistry registry,
         int numTargets) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 5.0, 5.0);
      setPelvisPoseHistoryCorrectorMaxVelocity(registry, 100.0, 100.0);
      
      RigidBodyTransform[] targets = createRandomCorrectionTargets(random, numTargets);
      boolean success = true;
      
      Vector4D error = new Vector4D();
      Vector3D targetTranslation = new Vector3D();
      RotationMatrix targetRotation = new RotationMatrix();
      double targetYaw = 0;
      double translationFudgeFactor = 0.01;
      double rotationFudgeFactor = 0.035;
      double largestError = Double.NEGATIVE_INFINITY;
      
      for (int i = 0; i < targets.length; i++)
      {
         targetTranslation.set(targets[i].getTranslation());
         targetRotation.set(targets[i].getRotation());
         targetYaw = targetRotation.getYaw();
         error.set(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ(), targetYaw);
         
         robotPose.getTranslation().set((double) i, (double) i, (double) i);
         
         long timeStamp = Conversions.secondsToNanoseconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = HumanoidMessageTools.createStampedPosePacket("/pelvis", timeStampedTransform, 1.0);
         
         success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 3);
         externalPelvisPoseCreator.setNewestPose(posePacket);
         
         while (translationClippedAlphaValue.getDoubleValue() > 0.2)
         {
            success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 3);
         }
         
         while (translationClippedAlphaValue.getDoubleValue() < 0.9999999)
         {
            success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 100);
         }
         
         double xError = Math.abs(correctedPelvis_x.getDoubleValue() - error.getX());
         double yError = Math.abs(correctedPelvis_y.getDoubleValue() - error.getY());
         double zError = Math.abs(correctedPelvis_z.getDoubleValue() - error.getZ());
         double yawError = Math.abs(correctedPelvis_yaw.getDoubleValue() - error.getS());
         
         if (xError > largestError)
            largestError = xError;
         if (yError > largestError)
            largestError = yError;
         if (zError > largestError)
            largestError = zError;
         
         success &= xError <= translationFudgeFactor;
         success &= yError <= translationFudgeFactor;
         success &= zError <= translationFudgeFactor;
         success &= MathTools.epsilonEquals(Math.abs(targetYaw), yawError, rotationFudgeFactor); // here, rotation error are not exectued, so the error should be the same as the target
         
         error.set(xError, yError, zError, yawError);
      }
      System.out.println(" max fudge factor: " + largestError);
      return success;
   }

   private boolean testYawForTranslationAndRotation(Random random, ExternalPelvisPoseCreator externalPelvisPoseCreator, YoRegistry registry, int numTargets)
         throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 5.0, 5.0);
      setPelvisPoseHistoryCorrectorMaxVelocity(registry, 100.0, 100.0);

      RigidBodyTransform[] targets = createYawOnlyCorrectionTargets(random, numTargets);
      boolean success = true;

      Vector4D error = new Vector4D();
      Vector3D targetTranslation = new Vector3D();
      RotationMatrix targetRotation = new RotationMatrix();
      double targetYaw = 0;
      double translationFudgeFactor = 0.01;
      double rotationFudgeFactor = 0.01;
      double largestError = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < targets.length; i++)
      {
         targets[i].getTranslation().set((double) i, (double) i, (double) (i / numTargets));
         targetTranslation.set(targets[i].getTranslation());
         targetRotation.set(targets[i].getRotation());
         targetYaw = targetRotation.getYaw();
         error.set(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ(), targetYaw);

         robotPose.getTranslation().set((double) i, (double) i, (double) (i / numTargets));

         long timeStamp = Conversions.secondsToNanoseconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = HumanoidMessageTools.createStampedPosePacket("/pelvis", timeStampedTransform, 1.0);

         success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 3);
         externalPelvisPoseCreator.setNewestPose(posePacket);

         while (translationClippedAlphaValue.getDoubleValue() > 0.2 || rotationClippedAlphaValue.getDoubleValue() > 0.2)
         {
            success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 3);
         }

         while (translationClippedAlphaValue.getDoubleValue() < 0.9999999 && rotationClippedAlphaValue.getDoubleValue() < 0.9999999)
         {
            success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 100);
         }

         double xError = Math.abs(correctedPelvis_x.getDoubleValue() - error.getX());
         double yError = Math.abs(correctedPelvis_y.getDoubleValue() - error.getY());
         double zError = Math.abs(correctedPelvis_z.getDoubleValue() - error.getZ());
         double yawError = Math.abs(correctedPelvis_yaw.getDoubleValue() - error.getS());

         if (xError > largestError)
            largestError = xError;
         if (yError > largestError)
            largestError = yError;
         if (zError > largestError)
            largestError = zError;
         if (yawError > largestError)
            largestError = yawError;

         success &= xError <= translationFudgeFactor;
         success &= yError <= translationFudgeFactor;
         success &= zError <= translationFudgeFactor;
         success &= yawError <= rotationFudgeFactor;

         error.set(xError, yError, zError, yawError);
      }
      System.out.println(" max fudge factor: " + largestError);
      return success;
   }
   
   private boolean testYawForTranslation(Random random, ExternalPelvisPoseCreator externalPelvisPoseCreator, YoRegistry registry, int numTargets)
         throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 5.0, 5.0);
      setPelvisPoseHistoryCorrectorMaxVelocity(registry, 100.0, 100.0);
      
      RigidBodyTransform[] targets = createYawOnlyCorrectionTargets(random, numTargets);
      boolean success = true;
      
      Vector4D error = new Vector4D();
      Vector3D targetTranslation = new Vector3D();
      RotationMatrix targetRotation = new RotationMatrix();
      double targetYaw = 0;
      double translationFudgeFactor = 0.01;
      double rotationFudgeFactor = 0.035;
      double largestError = Double.NEGATIVE_INFINITY;
      
      for (int i = 0; i < targets.length; i++)
      {
         targets[i].getTranslation().set((double) i, (double) i, (double) (i / numTargets));
         targetTranslation.set(targets[i].getTranslation());
         targetRotation.set(targets[i].getRotation());
         targetYaw = targetRotation.getYaw();
         error.set(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ(), targetYaw);
         
         robotPose.getTranslation().set((double) i, (double) i, (double) (i / numTargets));
         
         long timeStamp = Conversions.secondsToNanoseconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = HumanoidMessageTools.createStampedPosePacket("/pelvis", timeStampedTransform, 1.0);
         
         success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 3);
         externalPelvisPoseCreator.setNewestPose(posePacket);
         
         while (translationClippedAlphaValue.getDoubleValue() > 0.2)
         {
            success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 3);
         }
         
         while (translationClippedAlphaValue.getDoubleValue() < 0.9999999)
         {
            success &= simulateAndBlockAndCatchExceptions(Conversions.nanosecondsToSeconds(1000000) * 100);
         }
         
         double xError = Math.abs(correctedPelvis_x.getDoubleValue() - error.getX());
         double yError = Math.abs(correctedPelvis_y.getDoubleValue() - error.getY());
         double zError = Math.abs(correctedPelvis_z.getDoubleValue() - error.getZ());
         double yawError = Math.abs(correctedPelvis_yaw.getDoubleValue() - error.getS());
         
         if (xError > largestError)
            largestError = xError;
         if (yError > largestError)
            largestError = yError;
         if (zError > largestError)
            largestError = zError;
         
         success &= xError <= translationFudgeFactor;
         success &= yError <= translationFudgeFactor;
         success &= zError <= translationFudgeFactor;
         success &= MathTools.epsilonEquals(Math.abs(targetYaw), yawError, rotationFudgeFactor); // here, rotation error are not exectued, so the error should be the same as the target
         
         error.set(xError, yError, zError, yawError);
      }
      System.out.println(" max fudge factor: " + largestError);
      return success;
   }

   private void setPelvisPoseHistoryCorrectorAlphaBreakFreq(YoRegistry registry, double translationBreakFrequency,double rotationBreakFrequency)
   {
      YoDouble pelvisTranslationCorrectorAlphaFilterBF = (YoDouble) registry.findVariable("PelvisPoseHistoryCorrection",
            "interpolationTranslationAlphaFilterBreakFrequency");
      pelvisTranslationCorrectorAlphaFilterBF.set(translationBreakFrequency);
      
      YoDouble pelvisRotationCorrectorAlphaFilterBF = (YoDouble) registry.findVariable("PelvisPoseHistoryCorrection",
            "interpolationRotationAlphaFilterBreakFrequency");
      pelvisRotationCorrectorAlphaFilterBF.set(rotationBreakFrequency);
   }

   private void setPelvisPoseHistoryCorrectorMaxVelocity(YoRegistry registry, double maxTranslationVelocity, double maxRotationVelocity)
   {
      YoDouble maxTranslationVelocityCap = (YoDouble) registry.findVariable("PelvisPoseHistoryCorrection", "maxTranslationVelocityClip");
      maxTranslationVelocityCap.set(maxTranslationVelocity);
      YoDouble maxRotationVelocityCap = (YoDouble) registry.findVariable("PelvisPoseHistoryCorrection", "maxRotationVelocityClip");
      maxRotationVelocityCap.set(maxRotationVelocity);
   }

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
      public void sendPelvisPoseErrorPacket(
           PelvisPoseErrorPacket pelvisPoseErrorPacket) 
      {
        //doNothing
      }

      @Override
      public void sendLocalizationResetRequest(LocalizationPacket localizationPacket)
      {
         
      }
   }

   public boolean simulateAndBlockAndCatchExceptions(double simulationTime) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      try
      {
         bsr.simulateAndBlock(simulationTime);
         return true;
      }
      catch (SimulationExceededMaximumTimeException | ControllerFailureException e)
      {
         PrintTools.error(this, e.getMessage());
         throw e;
      }
   }

   private class PelvisPoseHistoryCorrectorController implements RobotController
   {

      private final PelvisPoseHistoryCorrection pelvisPoseHistoryCorrection;
      private final SimulationConstructionSet scs;
      private YoRegistry controllerRegistry = new YoRegistry(getName());

      public PelvisPoseHistoryCorrectorController(PelvisPoseHistoryCorrection pphc, SimulationConstructionSet scs)
      {
         pelvisPoseHistoryCorrection = pphc;
         this.scs = scs;
      }

      @Override
      public void initialize()
      {

      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return controllerRegistry;
      }

      @Override
      public String getName()
      {
         return "PelvisPoseHistoryCorrectorController";
      }

      @Override
      public String getDescription()
      {
         return "PelvisPoseHistoryCorrectorController";
      }

      @Override
      public void doControl()
      {
         refFrame.update();
         sixDofPelvisJoint.setJointConfiguration(robotPose);
         pelvisPoseHistoryCorrection.doControl(Conversions.secondsToNanoseconds(scs.getTime()));
         floatingJoint.setQuaternion(sixDofPelvisJoint.getJointPose().getOrientation());
         floatingJoint.setPosition(sixDofPelvisJoint.getJointPose().getPosition());
      }
   }

   private class RectangleRobot extends Robot
   {
      FloatingJoint baseJoint;
      public RectangleRobot()
      {
         super("RectangleRobot");
         baseJoint = new FloatingJoint("base", new Vector3D(0.0, 0.0, 0.0), this);
         Link link = base("base");
         baseJoint.setLink(link);
         this.addRootJoint(baseJoint);
      }

      private Link base(String name)
      {
         Link ret = new Link(name);
         ret.setMass(0.1);
         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.addCoordinateSystem(0.5);
         ret.setLinkGraphics(linkGraphics);
         return ret;
      }

   }
}
