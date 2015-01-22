package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

import org.junit.Test;

import us.ihmc.communication.packets.StampedPosePacket;
import us.ihmc.communication.packets.sensing.PelvisPoseErrorPacket;
import us.ihmc.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.PelvisPoseHistoryCorrection;

public class PelvisPoseHistoryCorrectionUsingSimpleRobotTest
{
   private static final boolean showGUI = false;

   private YoVariableRegistry registry;
   private RectangleRobot robot;
   private SimulationConstructionSet simulationConstructionSet;
   private RigidBodyTransform robotPose = new RigidBodyTransform();

   private DoubleYoVariable confidenceFactor; // target for alpha filter
   private DoubleYoVariable interpolationAlphaFilterBreakFrequency;
   private DoubleYoVariable interpolationAlphaFilterAlphaValue;
   private DoubleYoVariable interpolationAlphaFilter;
   private LongYoVariable seNonProcessedPelvisTimeStamp;
   private DoubleYoVariable maxVelocityClip;
   private DoubleYoVariable clippedAlphaValue;

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
   private DoubleYoVariable totalErrorFrame_x;
   private DoubleYoVariable totalErrorFrame_y;
   private DoubleYoVariable totalErrorFrame_z;
   private DoubleYoVariable totalErrorFrame_yaw;
   private DoubleYoVariable totalErrorFrame_pitch;
   private DoubleYoVariable totalErrorFrame_roll;
   private DoubleYoVariable interpolatedCorrectionFrame_x;
   private DoubleYoVariable interpolatedCorrectionFrame_y;
   private DoubleYoVariable interpolatedCorrectionFrame_z;
   private DoubleYoVariable interpolatedCorrectionFrame_yaw;
   private DoubleYoVariable interpolatedCorrectionFrame_pitch;
   private DoubleYoVariable interpolatedCorrectionFrame_roll;
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

   private Random random = new Random();

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
      interpolationAlphaFilter = (DoubleYoVariable) registry.getVariable(nameSpace, "PelvisErrorCorrectionAlphaFilter");
      interpolationAlphaFilterAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationAlphaFilterAlphaValue");
      interpolationAlphaFilterBreakFrequency = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationAlphaFilterBreakFrequency");
      confidenceFactor = (DoubleYoVariable) registry.getVariable(nameSpace, "PelvisErrorCorrectionConfidenceFactor");
      seNonProcessedPelvisTimeStamp = (LongYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_timestamp");
      maxVelocityClip = (DoubleYoVariable) registry.getVariable(nameSpace, "maxVelocityClip");
      clippedAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "clippedAlphaValue");

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
      totalErrorFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorFrame_x");
      totalErrorFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorFrame_y");
      totalErrorFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorFrame_z");
      totalErrorFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorFrame_yaw");
      totalErrorFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorFrame_pitch");
      totalErrorFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorFrame_roll");
      interpolatedCorrectionFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedCorrectionFrame_x");
      interpolatedCorrectionFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedCorrectionFrame_y");
      interpolatedCorrectionFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedCorrectionFrame_z");
      interpolatedCorrectionFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedCorrectionFrame_yaw");
      interpolatedCorrectionFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedCorrectionFrame_pitch");
      interpolatedCorrectionFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedCorrectionFrame_roll");
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
      simulationConstructionSet = new SimulationConstructionSet(robot, showGUI);
      simulationConstructionSet.setDT(0.001, 1);
      bsr = new BlockingSimulationRunner(simulationConstructionSet, 60.0 * 10.0);
      Thread simThread = new Thread(simulationConstructionSet);
      simThread.start();
   }

   private void setupCorrector()
   {
      externalPelvisPoseCreator = new ExternalPelvisPoseCreator();
      PelvisPoseHistoryCorrection pelvisCorrector = new PelvisPoseHistoryCorrection(sixDofPelvisJoint, TimeTools.nanoSecondstoSeconds(1000000), registry, 100,
            externalPelvisPoseCreator);
      PelvisPoseHistoryCorrectorController robotController = new PelvisPoseHistoryCorrectorController(pelvisCorrector, simulationConstructionSet);
      robot.setController(robotController);
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 0.015);
   }

   @Test(timeout=300000)
   public void testRandomInterpolationFinals() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      setupRobot();
      setupSim();
      setupCorrector();
      setupYoVariables(registry, "PelvisPoseHistoryCorrection");
      assertTrue(testInterpolationToRandomTargetsFromOrigin(externalPelvisPoseCreator, registry, 10));
      assertTrue(testInterpolationToRandomTargetsFromSpecificLocations(externalPelvisPoseCreator, registry, 10));
      assertTrue(testYaw(externalPelvisPoseCreator, registry, 10));
      BambooTools.reportTestFinishedMessage();
   }

   private RigidBodyTransform[] createRandomCorrectionTargets(int numOfTargets)
   {
      RigidBodyTransform[] targets = new RigidBodyTransform[numOfTargets];
      for (int i = 0; i < 10; i++)
      {
         targets[i] = RandomTools.generateRandomTransform(random);
      }
      return targets;
   }

   private RigidBodyTransform[] createYawOnlyCorrectionTargets(int numTargets)
   {
      RigidBodyTransform[] targets = new RigidBodyTransform[numTargets];
      Quat4d rot = new Quat4d();
      for (int i = 0; i < 10; i++)
      {
         targets[i] = new RigidBodyTransform();
         RotationFunctions.setQuaternionBasedOnYawPitchRoll(rot, (random.nextDouble() * Math.PI * 2) - Math.PI, 0, 0);
         targets[i].setRotation(rot);
      }
      return targets;

   }

   private boolean testInterpolationToRandomTargetsFromOrigin(ExternalPelvisPoseCreator externalPelvisPoseCreator, YoVariableRegistry registry, int numTargets)
         throws SimulationExceededMaximumTimeException
   {
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 5);
      setPelvisPoseHistoryCorrectorMaxVelocity(registry, 100);

      RigidBodyTransform[] targets = createRandomCorrectionTargets(numTargets);
      boolean success = true;

      Vector4d error = new Vector4d();
      Vector3d targetTranslation = new Vector3d();
      Matrix3d targetRotation = new Matrix3d();
      double targetYaw = 0;
      double translationFudgeFactor = 0.01;
      double rotationFudgeFactor = 0.035;
      double largestError = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < targets.length; i++)
      {
         targets[i].get(targetTranslation);
         targets[i].get(targetRotation);
         targetYaw = RotationFunctions.getYaw(targetRotation);
         error.set(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ(), targetYaw);

         long timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);

         success &= simulateAndBlockAndCatchExceptions(TimeTools.nanoSecondstoSeconds(1000000) * 3);
         externalPelvisPoseCreator.setNewestPose(posePacket);

         while (clippedAlphaValue.getDoubleValue() > 0.2)
         {
            success &= simulateAndBlockAndCatchExceptions(TimeTools.nanoSecondstoSeconds(1000000) * 3);
         }

         while (clippedAlphaValue.getDoubleValue() < 0.9999999)
         {
            success &= simulateAndBlockAndCatchExceptions(TimeTools.nanoSecondstoSeconds(1000000) * 100);
            ;
         }

         double xError = Math.abs(correctedPelvis_x.getDoubleValue() - error.getX());
         double yError = Math.abs(correctedPelvis_y.getDoubleValue() - error.getY());
         double zError = Math.abs(correctedPelvis_z.getDoubleValue() - error.getZ());
         double yawError = Math.abs(correctedPelvis_yaw.getDoubleValue() - error.getW());

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

   private boolean testInterpolationToRandomTargetsFromSpecificLocations(ExternalPelvisPoseCreator externalPelvisPoseCreator, YoVariableRegistry registry,
         int numTargets) throws SimulationExceededMaximumTimeException
   {
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 5);
      setPelvisPoseHistoryCorrectorMaxVelocity(registry, 100);

      RigidBodyTransform[] targets = createRandomCorrectionTargets(numTargets);
      boolean success = true;

      Vector4d error = new Vector4d();
      Vector3d targetTranslation = new Vector3d();
      Matrix3d targetRotation = new Matrix3d();
      double targetYaw = 0;
      double translationFudgeFactor = 0.01;
      double rotationFudgeFactor = 0.035;
      double largestError = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < targets.length; i++)
      {
         targets[i].get(targetTranslation);
         targets[i].get(targetRotation);
         targetYaw = RotationFunctions.getYaw(targetRotation);
         error.set(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ(), targetYaw);

         robotPose.setTranslation(i, i, i);

         long timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);

         success &= simulateAndBlockAndCatchExceptions(TimeTools.nanoSecondstoSeconds(1000000) * 3);
         externalPelvisPoseCreator.setNewestPose(posePacket);

         while (clippedAlphaValue.getDoubleValue() > 0.2)
         {
            success &= simulateAndBlockAndCatchExceptions(TimeTools.nanoSecondstoSeconds(1000000) * 3);
         }

         while (clippedAlphaValue.getDoubleValue() < 0.9999999)
         {
            success &= simulateAndBlockAndCatchExceptions(TimeTools.nanoSecondstoSeconds(1000000) * 100);
            ;
         }

         double xError = Math.abs(correctedPelvis_x.getDoubleValue() - error.getX());
         double yError = Math.abs(correctedPelvis_y.getDoubleValue() - error.getY());
         double zError = Math.abs(correctedPelvis_z.getDoubleValue() - error.getZ());
         double yawError = Math.abs(correctedPelvis_yaw.getDoubleValue() - error.getW());

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

   private boolean testYaw(ExternalPelvisPoseCreator externalPelvisPoseCreator, YoVariableRegistry registry, int numTargets)
         throws SimulationExceededMaximumTimeException
   {
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 5);
      setPelvisPoseHistoryCorrectorMaxVelocity(registry, 100);

      RigidBodyTransform[] targets = createYawOnlyCorrectionTargets(numTargets);
      boolean success = true;

      Vector4d error = new Vector4d();
      Vector3d targetTranslation = new Vector3d();
      Matrix3d targetRotation = new Matrix3d();
      double targetYaw = 0;
      double translationFudgeFactor = 0.01;
      double rotationFudgeFactor = 0.035;
      double largestError = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < targets.length; i++)
      {
         targets[i].setTranslation(i, i, i / numTargets);
         targets[i].get(targetTranslation);
         targets[i].get(targetRotation);
         targetYaw = RotationFunctions.getYaw(targetRotation);
         error.set(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ(), targetYaw);

         robotPose.setTranslation(i, i, i / numTargets);

         long timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);

         success &= simulateAndBlockAndCatchExceptions(TimeTools.nanoSecondstoSeconds(1000000) * 3);
         externalPelvisPoseCreator.setNewestPose(posePacket);

         while (clippedAlphaValue.getDoubleValue() > 0.2)
         {
            success &= simulateAndBlockAndCatchExceptions(TimeTools.nanoSecondstoSeconds(1000000) * 3);
         }

         while (clippedAlphaValue.getDoubleValue() < 0.9999999)
         {
            success &= simulateAndBlockAndCatchExceptions(TimeTools.nanoSecondstoSeconds(1000000) * 100);
            ;
         }

         double xError = Math.abs(correctedPelvis_x.getDoubleValue() - error.getX());
         double yError = Math.abs(correctedPelvis_y.getDoubleValue() - error.getY());
         double zError = Math.abs(correctedPelvis_z.getDoubleValue() - error.getZ());
         double yawError = Math.abs(correctedPelvis_yaw.getDoubleValue() - error.getW());

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

   private void setPelvisPoseHistoryCorrectorAlphaBreakFreq(YoVariableRegistry registry, double breakFrequency)
   {
      DoubleYoVariable pelvisCorrectorAlphaFilterBF = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection",
            "interpolationAlphaFilterBreakFrequency");
      pelvisCorrectorAlphaFilterBF.set(breakFrequency);
   }

   private void setPelvisPoseHistoryCorrectorMaxVelocity(YoVariableRegistry registry, double maxVel)
   {
      DoubleYoVariable maxVelocityCap = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "maxVelocityClip");
      maxVelocityCap.set(maxVel);
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
   }

   public boolean simulateAndBlockAndCatchExceptions(double simulationTime) throws SimulationExceededMaximumTimeException
   {
      try
      {
         bsr.simulateAndBlock(simulationTime);
         return true;
      }
      catch (Exception e)
      {
         System.err.println("Caught exception in SimulationTestHelper.simulateAndBlockAndCatchExceptions. Exception = /n" + e);
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
         pelvisPoseHistoryCorrection.doControl(TimeTools.secondsToNanoSeconds(scs.getTime()));
         floatingJoint.setRotationAndTranslation(sixDofPelvisJoint.getJointTransform3D());
      }
   }

   private class RectangleRobot extends Robot
   {
      FloatingJoint baseJoint;
      public RectangleRobot()
      {
         super("RectangleRobot");
         baseJoint = new FloatingJoint("base", new Vector3d(0.0, 0.0, 0.0), this);
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
