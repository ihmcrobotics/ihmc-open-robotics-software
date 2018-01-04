package us.ihmc.avatar.stateEstimationEndToEndTests;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PelvisPoseErrorPacket;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.PelvisPoseHistoryCorrection;
import us.ihmc.tools.MemoryTools;

public class PelvisPoseHistoryCorrectionUsingSimpleRobotTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

//   private static final boolean showGUI = false;
   //TODO get that from the StateEstimatorParameters
   private static final boolean USE_ROTATION_CORRECTION = false;
   
   private YoVariableRegistry registry;
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
            System.out.println(vars[i] + postFix[j] + " = (YoDouble) registry.getVariable(nameSpace, \"" + vars[i] + postFix[j] + "\"); ");
         }
      }
   }

   private void setupYoVariables(YoVariableRegistry registry, String nameSpace)
   {
      interpolationTranslationAlphaFilter = (YoDouble) registry.getVariable(nameSpace, "PelvisTranslationErrorCorrectionAlphaFilter");
      interpolationTranslationAlphaFilterAlphaValue = (YoDouble) registry.getVariable(nameSpace, "interpolationTranslationAlphaFilterAlphaValue");
      interpolationTranslationAlphaFilterBreakFrequency = (YoDouble) registry.getVariable(nameSpace, "interpolationTranslationAlphaFilterBreakFrequency");
      interpolationRotationAlphaFilter = (YoDouble) registry.getVariable(nameSpace, "PelvisRotationErrorCorrectionAlphaFilter");
      interpolationRotationAlphaFilterAlphaValue = (YoDouble) registry.getVariable(nameSpace, "interpolationRotationAlphaFilterAlphaValue");
      interpolationRotationAlphaFilterBreakFrequency = (YoDouble) registry.getVariable(nameSpace, "interpolationRotationAlphaFilterBreakFrequency");
      confidenceFactor = (YoDouble) registry.getVariable(nameSpace, "PelvisErrorCorrectionConfidenceFactor");
      seNonProcessedPelvisTimeStamp = (YoLong) registry.getVariable(nameSpace, "seNonProcessedPelvis_timestamp");
      maxTranslationVelocityClip = (YoDouble) registry.getVariable(nameSpace, "maxTranslationVelocityClip");
      maxRotationVelocityClip = (YoDouble) registry.getVariable(nameSpace, "maxRotationVelocityClip");
      translationClippedAlphaValue = (YoDouble) registry.getVariable(nameSpace, "translationClippedAlphaValue");
      rotationClippedAlphaValue = (YoDouble) registry.getVariable(nameSpace, "rotationClippedAlphaValue");

      nonCorrectedPelvis_x = (YoDouble) registry.getVariable(nameSpace, "nonCorrectedPelvis_x");
      nonCorrectedPelvis_y = (YoDouble) registry.getVariable(nameSpace, "nonCorrectedPelvis_y");
      nonCorrectedPelvis_z = (YoDouble) registry.getVariable(nameSpace, "nonCorrectedPelvis_z");
      nonCorrectedPelvis_yaw = (YoDouble) registry.getVariable(nameSpace, "nonCorrectedPelvis_yaw");
      nonCorrectedPelvis_pitch = (YoDouble) registry.getVariable(nameSpace, "nonCorrectedPelvis_pitch");
      nonCorrectedPelvis_roll = (YoDouble) registry.getVariable(nameSpace, "nonCorrectedPelvis_roll");
      correctedPelvis_x = (YoDouble) registry.getVariable(nameSpace, "correctedPelvis_x");
      correctedPelvis_y = (YoDouble) registry.getVariable(nameSpace, "correctedPelvis_y");
      correctedPelvis_z = (YoDouble) registry.getVariable(nameSpace, "correctedPelvis_z");
      correctedPelvis_yaw = (YoDouble) registry.getVariable(nameSpace, "correctedPelvis_yaw");
      correctedPelvis_pitch = (YoDouble) registry.getVariable(nameSpace, "correctedPelvis_pitch");
      correctedPelvis_roll = (YoDouble) registry.getVariable(nameSpace, "correctedPelvis_roll");
      seBackInTimeFrame_x = (YoDouble) registry.getVariable(nameSpace, "seBackInTimeFrame_x");
      seBackInTimeFrame_y = (YoDouble) registry.getVariable(nameSpace, "seBackInTimeFrame_y");
      seBackInTimeFrame_z = (YoDouble) registry.getVariable(nameSpace, "seBackInTimeFrame_z");
      seBackInTimeFrame_yaw = (YoDouble) registry.getVariable(nameSpace, "seBackInTimeFrame_yaw");
      seBackInTimeFrame_pitch = (YoDouble) registry.getVariable(nameSpace, "seBackInTimeFrame_pitch");
      seBackInTimeFrame_roll = (YoDouble) registry.getVariable(nameSpace, "seBackInTimeFrame_roll");
      localizationBackInTimeFrame_x = (YoDouble) registry.getVariable(nameSpace, "localizationBackInTimeFrame_x");
      localizationBackInTimeFrame_y = (YoDouble) registry.getVariable(nameSpace, "localizationBackInTimeFrame_y");
      localizationBackInTimeFrame_z = (YoDouble) registry.getVariable(nameSpace, "localizationBackInTimeFrame_z");
      localizationBackInTimeFrame_yaw = (YoDouble) registry.getVariable(nameSpace, "localizationBackInTimeFrame_yaw");
      localizationBackInTimeFrame_pitch = (YoDouble) registry.getVariable(nameSpace, "localizationBackInTimeFrame_pitch");
      localizationBackInTimeFrame_roll = (YoDouble) registry.getVariable(nameSpace, "localizationBackInTimeFrame_roll");
      
      totalTranslationErrorFrame_x = (YoDouble) registry.getVariable(nameSpace, "totalTranslationErrorFrame_x");
      totalTranslationErrorFrame_y = (YoDouble) registry.getVariable(nameSpace, "totalTranslationErrorFrame_y");
      totalTranslationErrorFrame_z = (YoDouble) registry.getVariable(nameSpace, "totalTranslationErrorFrame_z");
      totalTranslationErrorFrame_yaw = (YoDouble) registry.getVariable(nameSpace, "totalTranslationErrorFrame_yaw");
      totalTranslationErrorFrame_pitch = (YoDouble) registry.getVariable(nameSpace, "totalTranslationErrorFrame_pitch");
      totalTranslationErrorFrame_roll = (YoDouble) registry.getVariable(nameSpace, "totalTranslationErrorFrame_roll");
      
      totalRotationErrorFrame_x = (YoDouble) registry.getVariable(nameSpace, "totalRotationErrorFrame_x");
      totalRotationErrorFrame_y = (YoDouble) registry.getVariable(nameSpace, "totalRotationErrorFrame_y");
      totalRotationErrorFrame_z = (YoDouble) registry.getVariable(nameSpace, "totalRotationErrorFrame_z");
      totalRotationErrorFrame_yaw = (YoDouble) registry.getVariable(nameSpace, "totalRotationErrorFrame_yaw");
      totalRotationErrorFrame_pitch = (YoDouble) registry.getVariable(nameSpace, "totalRotationErrorFrame_pitch");
      totalRotationErrorFrame_roll = (YoDouble) registry.getVariable(nameSpace, "totalRotationErrorFrame_roll");
      
      interpolatedTranslationCorrectionFrame_x = (YoDouble) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_x");
      interpolatedTranslationCorrectionFrame_y = (YoDouble) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_y");
      interpolatedTranslationCorrectionFrame_z = (YoDouble) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_z");
      interpolatedTranslationCorrectionFrame_yaw = (YoDouble) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_yaw");
      interpolatedTranslationCorrectionFrame_pitch = (YoDouble) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_pitch");
      interpolatedTranslationCorrectionFrame_roll = (YoDouble) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_roll");
      
      interpolatedRotationCorrectionFrame_x = (YoDouble) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_x");
      interpolatedRotationCorrectionFrame_y = (YoDouble) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_y");
      interpolatedRotationCorrectionFrame_z = (YoDouble) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_z");
      interpolatedRotationCorrectionFrame_yaw = (YoDouble) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_yaw");
      interpolatedRotationCorrectionFrame_pitch = (YoDouble) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_pitch");
      interpolatedRotationCorrectionFrame_roll = (YoDouble) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_roll");
      
      interpolationTranslationStartFrame_x = (YoDouble) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_x");
      interpolationTranslationStartFrame_y = (YoDouble) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_y");
      interpolationTranslationStartFrame_z = (YoDouble) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_z");
      interpolationTranslationStartFrame_yaw = (YoDouble) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_yaw");
      interpolationTranslationStartFrame_pitch = (YoDouble) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_pitch");
      interpolationTranslationStartFrame_roll = (YoDouble) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_roll");
      
      interpolationRotationStartFrame_x = (YoDouble) registry.getVariable(nameSpace, "interpolationRotationStartFrame_x");
      interpolationRotationStartFrame_y = (YoDouble) registry.getVariable(nameSpace, "interpolationRotationStartFrame_y");
      interpolationRotationStartFrame_z = (YoDouble) registry.getVariable(nameSpace, "interpolationRotationStartFrame_z");
      interpolationRotationStartFrame_yaw = (YoDouble) registry.getVariable(nameSpace, "interpolationRotationStartFrame_yaw");
      interpolationRotationStartFrame_pitch = (YoDouble) registry.getVariable(nameSpace, "interpolationRotationStartFrame_pitch");
      interpolationRotationStartFrame_roll = (YoDouble) registry.getVariable(nameSpace, "interpolationRotationStartFrame_roll");

   }

   private void setupRobot()
   {
      robot = new RectangleRobot();
      robot.setDynamic(false);
      robot.setGravity(0);
      registry = robot.getRobotsYoVariableRegistry();
      

      floatingJoint = (FloatingJoint) robot.getRootJoints().get(0);
      refFrame = new ReferenceFrame("pelvis", ReferenceFrame.getWorldFrame(), true, false)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(robotPose);
         }
      };
      RigidBody rigidBody = new RigidBody("pelvis", refFrame);
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

	@ContinuousIntegrationTest(estimatedDuration = 27.3)
	@Test(timeout = 140000)
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
         targets[i].setRotation(rot);
      }
      return targets;

   }

   private boolean testInterpolationForTranslationAndRotationToRandomTargetsFromOrigin(Random random, ExternalPelvisPoseCreator externalPelvisPoseCreator, YoVariableRegistry registry, int numTargets)
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
         targets[i].getTranslation(targetTranslation);
         targets[i].getRotation(targetRotation);
         targetYaw = targetRotation.getYaw();
         error.set(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ(), targetYaw);

         long timeStamp = Conversions.secondsToNanoseconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);

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
   
   private boolean testInterpolationForTranslationToRandomTargetsFromOrigin(Random random, ExternalPelvisPoseCreator externalPelvisPoseCreator, YoVariableRegistry registry, int numTargets)
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
         targets[i].getTranslation(targetTranslation);
         targets[i].getRotation(targetRotation);
         targetYaw = targetRotation.getYaw();
         error.set(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ(), targetYaw);
         
         long timeStamp = Conversions.secondsToNanoseconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);
         
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

   private boolean testInterpolationForTranslationAndRotationToRandomTargetsFromSpecificLocations(Random random, ExternalPelvisPoseCreator externalPelvisPoseCreator, YoVariableRegistry registry,
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
         targets[i].getTranslation(targetTranslation);
         targets[i].getRotation(targetRotation);
         targetYaw = targetRotation.getYaw();
         error.set(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ(), targetYaw);

         robotPose.setTranslation(i, i, i);

         long timeStamp = Conversions.secondsToNanoseconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);

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
   
   private boolean testInterpolationForTranslationToRandomTargetsFromSpecificLocations(Random random, ExternalPelvisPoseCreator externalPelvisPoseCreator, YoVariableRegistry registry,
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
         targets[i].getTranslation(targetTranslation);
         targets[i].getRotation(targetRotation);
         targetYaw = targetRotation.getYaw();
         error.set(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ(), targetYaw);
         
         robotPose.setTranslation(i, i, i);
         
         long timeStamp = Conversions.secondsToNanoseconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);
         
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

   private boolean testYawForTranslationAndRotation(Random random, ExternalPelvisPoseCreator externalPelvisPoseCreator, YoVariableRegistry registry, int numTargets)
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
         targets[i].setTranslation(i, i, i / numTargets);
         targets[i].getTranslation(targetTranslation);
         targets[i].getRotation(targetRotation);
         targetYaw = targetRotation.getYaw();
         error.set(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ(), targetYaw);

         robotPose.setTranslation(i, i, i / numTargets);

         long timeStamp = Conversions.secondsToNanoseconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);

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
   
   private boolean testYawForTranslation(Random random, ExternalPelvisPoseCreator externalPelvisPoseCreator, YoVariableRegistry registry, int numTargets)
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
         targets[i].setTranslation(i, i, i / numTargets);
         targets[i].getTranslation(targetTranslation);
         targets[i].getRotation(targetRotation);
         targetYaw = targetRotation.getYaw();
         error.set(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ(), targetYaw);
         
         robotPose.setTranslation(i, i, i / numTargets);
         
         long timeStamp = Conversions.secondsToNanoseconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);
         
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

   private void setPelvisPoseHistoryCorrectorAlphaBreakFreq(YoVariableRegistry registry, double translationBreakFrequency,double rotationBreakFrequency)
   {
      YoDouble pelvisTranslationCorrectorAlphaFilterBF = (YoDouble) registry.getVariable("PelvisPoseHistoryCorrection",
            "interpolationTranslationAlphaFilterBreakFrequency");
      pelvisTranslationCorrectorAlphaFilterBF.set(translationBreakFrequency);
      
      YoDouble pelvisRotationCorrectorAlphaFilterBF = (YoDouble) registry.getVariable("PelvisPoseHistoryCorrection",
            "interpolationRotationAlphaFilterBreakFrequency");
      pelvisRotationCorrectorAlphaFilterBF.set(rotationBreakFrequency);
   }

   private void setPelvisPoseHistoryCorrectorMaxVelocity(YoVariableRegistry registry, double maxTranslationVelocity, double maxRotationVelocity)
   {
      YoDouble maxTranslationVelocityCap = (YoDouble) registry.getVariable("PelvisPoseHistoryCorrection", "maxTranslationVelocityClip");
      maxTranslationVelocityCap.set(maxTranslationVelocity);
      YoDouble maxRotationVelocityCap = (YoDouble) registry.getVariable("PelvisPoseHistoryCorrection", "maxRotationVelocityClip");
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
      private YoVariableRegistry controllerRegistry = new YoVariableRegistry(getName());

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
      public YoVariableRegistry getYoVariableRegistry()
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
         sixDofPelvisJoint.setPositionAndRotation(robotPose);
         pelvisPoseHistoryCorrection.doControl(Conversions.secondsToNanoseconds(scs.getTime()));
         floatingJoint.setRotationAndTranslation(sixDofPelvisJoint.getJointTransform3D());
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
