package us.ihmc.avatar.stateEstimationEndToEndTests;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.RotationMatrix;
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
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionsettools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.PelvisPoseHistoryCorrection;
import us.ihmc.tools.MemoryTools;

public class PelvisPoseHistoryCorrectionUsingSimpleRobotTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

//   private static final boolean showGUI = false;
   //TODO get that from the StateEstimatorParameters
   private static final boolean USE_ROTATION_CORRECTION = false;
   
   private YoVariableRegistry registry;
   private RectangleRobot robot;
   private SimulationConstructionSet simulationConstructionSet;
   private RigidBodyTransform robotPose = new RigidBodyTransform();

   private DoubleYoVariable confidenceFactor; // target for alpha filter
   private DoubleYoVariable interpolationTranslationAlphaFilterBreakFrequency;
   private DoubleYoVariable interpolationTranslationAlphaFilterAlphaValue;
   private DoubleYoVariable interpolationTranslationAlphaFilter;
   private DoubleYoVariable interpolationRotationAlphaFilterBreakFrequency;
   private DoubleYoVariable interpolationRotationAlphaFilterAlphaValue;
   private DoubleYoVariable interpolationRotationAlphaFilter;
   private LongYoVariable seNonProcessedPelvisTimeStamp;
   private DoubleYoVariable maxTranslationVelocityClip;
   private DoubleYoVariable maxRotationVelocityClip;
   private DoubleYoVariable translationClippedAlphaValue;
   private DoubleYoVariable rotationClippedAlphaValue;

   private DoubleYoVariable nonCorrectedPelvis_x;
   private DoubleYoVariable nonCorrectedPelvis_y;
   private DoubleYoVariable nonCorrectedPelvis_z;
   private DoubleYoVariable nonCorrectedPelvis_yaw;
   private DoubleYoVariable nonCorrectedPelvis_pitch;
   private DoubleYoVariable nonCorrectedPelvis_roll;
   private DoubleYoVariable correctedPelvis_x;
   private DoubleYoVariable correctedPelvis_y;
   private DoubleYoVariable correctedPelvis_z;
   private DoubleYoVariable correctedPelvis_yaw;
   private DoubleYoVariable correctedPelvis_pitch;
   private DoubleYoVariable correctedPelvis_roll;
   private DoubleYoVariable seBackInTimeFrame_x;
   private DoubleYoVariable seBackInTimeFrame_y;
   private DoubleYoVariable seBackInTimeFrame_z;
   private DoubleYoVariable seBackInTimeFrame_yaw;
   private DoubleYoVariable seBackInTimeFrame_pitch;
   private DoubleYoVariable seBackInTimeFrame_roll;
   private DoubleYoVariable localizationBackInTimeFrame_x;
   private DoubleYoVariable localizationBackInTimeFrame_y;
   private DoubleYoVariable localizationBackInTimeFrame_z;
   private DoubleYoVariable localizationBackInTimeFrame_yaw;
   private DoubleYoVariable localizationBackInTimeFrame_pitch;
   private DoubleYoVariable localizationBackInTimeFrame_roll;
   
   private DoubleYoVariable totalTranslationErrorFrame_x;
   private DoubleYoVariable totalTranslationErrorFrame_y;
   private DoubleYoVariable totalTranslationErrorFrame_z;
   private DoubleYoVariable totalTranslationErrorFrame_yaw;
   private DoubleYoVariable totalTranslationErrorFrame_pitch;
   private DoubleYoVariable totalTranslationErrorFrame_roll;
   
   private DoubleYoVariable totalRotationErrorFrame_x;
   private DoubleYoVariable totalRotationErrorFrame_y;
   private DoubleYoVariable totalRotationErrorFrame_z;
   private DoubleYoVariable totalRotationErrorFrame_yaw;
   private DoubleYoVariable totalRotationErrorFrame_pitch;
   private DoubleYoVariable totalRotationErrorFrame_roll;
   
   private DoubleYoVariable interpolatedTranslationCorrectionFrame_x;
   private DoubleYoVariable interpolatedTranslationCorrectionFrame_y;
   private DoubleYoVariable interpolatedTranslationCorrectionFrame_z;
   private DoubleYoVariable interpolatedTranslationCorrectionFrame_yaw;
   private DoubleYoVariable interpolatedTranslationCorrectionFrame_pitch;
   private DoubleYoVariable interpolatedTranslationCorrectionFrame_roll;
   
   private DoubleYoVariable interpolatedRotationCorrectionFrame_x;
   private DoubleYoVariable interpolatedRotationCorrectionFrame_y;
   private DoubleYoVariable interpolatedRotationCorrectionFrame_z;
   private DoubleYoVariable interpolatedRotationCorrectionFrame_yaw;
   private DoubleYoVariable interpolatedRotationCorrectionFrame_pitch;
   private DoubleYoVariable interpolatedRotationCorrectionFrame_roll;
   
   private DoubleYoVariable interpolationTranslationStartFrame_x;
   private DoubleYoVariable interpolationTranslationStartFrame_y;
   private DoubleYoVariable interpolationTranslationStartFrame_z;
   private DoubleYoVariable interpolationTranslationStartFrame_yaw;
   private DoubleYoVariable interpolationTranslationStartFrame_pitch;
   private DoubleYoVariable interpolationTranslationStartFrame_roll;
   
   private DoubleYoVariable interpolationRotationStartFrame_x;
   private DoubleYoVariable interpolationRotationStartFrame_y;
   private DoubleYoVariable interpolationRotationStartFrame_z;
   private DoubleYoVariable interpolationRotationStartFrame_yaw;
   private DoubleYoVariable interpolationRotationStartFrame_pitch;
   private DoubleYoVariable interpolationRotationStartFrame_roll;

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
            System.out.println("private DoubleYoVariable " + vars[i] + postFix[j] + ";");
         }
      }
      for (int i = 0; i < vars.length; i++)
      {
         for (int j = 0; j < postFix.length; j++)
         {
            System.out.println(vars[i] + postFix[j] + " = (DoubleYoVariable) registry.getVariable(nameSpace, \"" + vars[i] + postFix[j] + "\"); ");
         }
      }
   }

   private void setupYoVariables(YoVariableRegistry registry, String nameSpace)
   {
      interpolationTranslationAlphaFilter = (DoubleYoVariable) registry.getVariable(nameSpace, "PelvisTranslationErrorCorrectionAlphaFilter");
      interpolationTranslationAlphaFilterAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationTranslationAlphaFilterAlphaValue");
      interpolationTranslationAlphaFilterBreakFrequency = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationTranslationAlphaFilterBreakFrequency");
      interpolationRotationAlphaFilter = (DoubleYoVariable) registry.getVariable(nameSpace, "PelvisRotationErrorCorrectionAlphaFilter");
      interpolationRotationAlphaFilterAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationRotationAlphaFilterAlphaValue");
      interpolationRotationAlphaFilterBreakFrequency = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationRotationAlphaFilterBreakFrequency");
      confidenceFactor = (DoubleYoVariable) registry.getVariable(nameSpace, "PelvisErrorCorrectionConfidenceFactor");
      seNonProcessedPelvisTimeStamp = (LongYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_timestamp");
      maxTranslationVelocityClip = (DoubleYoVariable) registry.getVariable(nameSpace, "maxTranslationVelocityClip");
      maxRotationVelocityClip = (DoubleYoVariable) registry.getVariable(nameSpace, "maxRotationVelocityClip");
      translationClippedAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "translationClippedAlphaValue");
      rotationClippedAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "rotationClippedAlphaValue");

      nonCorrectedPelvis_x = (DoubleYoVariable) registry.getVariable(nameSpace, "nonCorrectedPelvis_x");
      nonCorrectedPelvis_y = (DoubleYoVariable) registry.getVariable(nameSpace, "nonCorrectedPelvis_y");
      nonCorrectedPelvis_z = (DoubleYoVariable) registry.getVariable(nameSpace, "nonCorrectedPelvis_z");
      nonCorrectedPelvis_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "nonCorrectedPelvis_yaw");
      nonCorrectedPelvis_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "nonCorrectedPelvis_pitch");
      nonCorrectedPelvis_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "nonCorrectedPelvis_roll");
      correctedPelvis_x = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_x");
      correctedPelvis_y = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_y");
      correctedPelvis_z = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_z");
      correctedPelvis_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_yaw");
      correctedPelvis_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_pitch");
      correctedPelvis_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_roll");
      seBackInTimeFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "seBackInTimeFrame_x");
      seBackInTimeFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "seBackInTimeFrame_y");
      seBackInTimeFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "seBackInTimeFrame_z");
      seBackInTimeFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "seBackInTimeFrame_yaw");
      seBackInTimeFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "seBackInTimeFrame_pitch");
      seBackInTimeFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "seBackInTimeFrame_roll");
      localizationBackInTimeFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "localizationBackInTimeFrame_x");
      localizationBackInTimeFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "localizationBackInTimeFrame_y");
      localizationBackInTimeFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "localizationBackInTimeFrame_z");
      localizationBackInTimeFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "localizationBackInTimeFrame_yaw");
      localizationBackInTimeFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "localizationBackInTimeFrame_pitch");
      localizationBackInTimeFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "localizationBackInTimeFrame_roll");
      
      totalTranslationErrorFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "totalTranslationErrorFrame_x");
      totalTranslationErrorFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "totalTranslationErrorFrame_y");
      totalTranslationErrorFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "totalTranslationErrorFrame_z");
      totalTranslationErrorFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "totalTranslationErrorFrame_yaw");
      totalTranslationErrorFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "totalTranslationErrorFrame_pitch");
      totalTranslationErrorFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "totalTranslationErrorFrame_roll");
      
      totalRotationErrorFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "totalRotationErrorFrame_x");
      totalRotationErrorFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "totalRotationErrorFrame_y");
      totalRotationErrorFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "totalRotationErrorFrame_z");
      totalRotationErrorFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "totalRotationErrorFrame_yaw");
      totalRotationErrorFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "totalRotationErrorFrame_pitch");
      totalRotationErrorFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "totalRotationErrorFrame_roll");
      
      interpolatedTranslationCorrectionFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_x");
      interpolatedTranslationCorrectionFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_y");
      interpolatedTranslationCorrectionFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_z");
      interpolatedTranslationCorrectionFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_yaw");
      interpolatedTranslationCorrectionFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_pitch");
      interpolatedTranslationCorrectionFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_roll");
      
      interpolatedRotationCorrectionFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_x");
      interpolatedRotationCorrectionFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_y");
      interpolatedRotationCorrectionFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_z");
      interpolatedRotationCorrectionFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_yaw");
      interpolatedRotationCorrectionFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_pitch");
      interpolatedRotationCorrectionFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_roll");
      
      interpolationTranslationStartFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_x");
      interpolationTranslationStartFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_y");
      interpolationTranslationStartFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_z");
      interpolationTranslationStartFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_yaw");
      interpolationTranslationStartFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_pitch");
      interpolationTranslationStartFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_roll");
      
      interpolationRotationStartFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationRotationStartFrame_x");
      interpolationRotationStartFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationRotationStartFrame_y");
      interpolationRotationStartFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationRotationStartFrame_z");
      interpolationRotationStartFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationRotationStartFrame_yaw");
      interpolationRotationStartFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationRotationStartFrame_pitch");
      interpolationRotationStartFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationRotationStartFrame_roll");

   }

   private void setupRobot()
   {
      robot = new RectangleRobot();
      robot.setDynamic(false);
      robot.setGravity(0);
      registry = robot.getRobotsYoVariableRegistry();
      

      floatingJoint = (FloatingJoint) robot.getRootJoints().get(0);
      refFrame = new ReferenceFrame("pelvis", ReferenceFrame.getWorldFrame())
      {
         private static final long serialVersionUID = -6427490298776551499L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(robotPose);
         }
      };
      RigidBody rigidBody = new RigidBody("pelvis", refFrame);
      sixDofPelvisJoint = new SixDoFJoint("pelvis", rigidBody, refFrame);
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
         targets[i] = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
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
      DoubleYoVariable pelvisTranslationCorrectorAlphaFilterBF = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection",
            "interpolationTranslationAlphaFilterBreakFrequency");
      pelvisTranslationCorrectorAlphaFilterBF.set(translationBreakFrequency);
      
      DoubleYoVariable pelvisRotationCorrectorAlphaFilterBF = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection",
            "interpolationRotationAlphaFilterBreakFrequency");
      pelvisRotationCorrectorAlphaFilterBF.set(rotationBreakFrequency);
   }

   private void setPelvisPoseHistoryCorrectorMaxVelocity(YoVariableRegistry registry, double maxTranslationVelocity, double maxRotationVelocity)
   {
      DoubleYoVariable maxTranslationVelocityCap = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "maxTranslationVelocityClip");
      maxTranslationVelocityCap.set(maxTranslationVelocity);
      DoubleYoVariable maxRotationVelocityCap = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "maxRotationVelocityClip");
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
         // TODO Auto-generated method stub
         
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
