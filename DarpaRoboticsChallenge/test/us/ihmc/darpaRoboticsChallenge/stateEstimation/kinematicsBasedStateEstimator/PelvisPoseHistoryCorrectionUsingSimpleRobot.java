package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.communication.packets.StampedPosePacket;
import us.ihmc.communication.subscribers.ExternalPelvisPoseSubscriberInterface;
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

import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.Link;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class PelvisPoseHistoryCorrectionUsingSimpleRobot
{
   private static final boolean showGUI = true;//KEEP_SCS_UP || createMovie;

   private YoVariableRegistry registry;
   private RectangleRobot robot;
   private SimulationConstructionSet simulationConstructionSet;
   private RigidBodyTransform robotPose = new RigidBodyTransform();

   private DoubleYoVariable seNonProcessedPelvisQuaternionX;
   private DoubleYoVariable seNonProcessedPelvisQuaternionY;
   private DoubleYoVariable seNonProcessedPelvisQuaternionZ;
   private DoubleYoVariable seNonProcessedPelvisQuaternionS;

   private DoubleYoVariable confidenceFactor; // target for alpha filter
   private DoubleYoVariable interpolationAlphaFilterBreakFrequency;
   private DoubleYoVariable interpolationAlphaFilterAlphaValue;
   private DoubleYoVariable interpolationAlphaFilter;

   private DoubleYoVariable externalPelvisPositionX;
   private DoubleYoVariable externalPelvisPositionY;
   private DoubleYoVariable externalPelvisPositionZ;
   private DoubleYoVariable externalPelvisPitch;
   private DoubleYoVariable externalPelvisRoll;
   private DoubleYoVariable externalPelvisYaw;

   private DoubleYoVariable totalErrorPelvisPositionX;
   private DoubleYoVariable totalErrorPelvisPositionY;
   private DoubleYoVariable totalErrorPelvisPositionZ;
   private DoubleYoVariable totalErrorPelvisPitch;
   private DoubleYoVariable totalErrorPelvisRoll;
   private DoubleYoVariable totalErrorPelvisYaw;

   private DoubleYoVariable interpolatedPelvisErrorPositionX;
   private DoubleYoVariable interpolatedPelvisErrorPositionY;
   private DoubleYoVariable interpolatedPelvisErrorPositionZ;
   private DoubleYoVariable interpolatedPelvisErrorPitch;
   private DoubleYoVariable interpolatedPelvisErrorRoll;
   private DoubleYoVariable interpolatedPelvisErrorYaw;

   private DoubleYoVariable previousInterpolatedPelvisErrorPositionX;
   private DoubleYoVariable previousInterpolatedPelvisErrorPositionY;
   private DoubleYoVariable previousInterpolatedPelvisErrorPositionZ;
   private DoubleYoVariable previousInterpolatedPelvisErrorPitch;
   private DoubleYoVariable previousInterpolatedPelvisErrorRoll;
   private DoubleYoVariable previousInterpolatedPelvisErrorYaw;

   private DoubleYoVariable seNonProcessedPelvisPositionX;
   private DoubleYoVariable seNonProcessedPelvisPositionY;
   private DoubleYoVariable seNonProcessedPelvisPositionZ;
   private DoubleYoVariable seNonProcessedPelvisPitch;
   private DoubleYoVariable seNonProcessedPelvisRoll;
   private DoubleYoVariable seNonProcessedPelvisYaw;
   private LongYoVariable seNonProcessedPelvisTimeStamp;

   private DoubleYoVariable correctedPelvisPositionX;
   private DoubleYoVariable correctedPelvisPositionY;
   private DoubleYoVariable correctedPelvisPositionZ;
   private DoubleYoVariable correctedPelvisPitch;
   private DoubleYoVariable correctedPelvisRoll;
   private DoubleYoVariable correctedPelvisYaw;

   private DoubleYoVariable maxVelocityClip;
   private DoubleYoVariable clippedAlphaValue;

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

   private void setupYoVariables(YoVariableRegistry registry, String nameSpace)
   {
      interpolationAlphaFilter = (DoubleYoVariable) registry.getVariable(nameSpace, "PelvisErrorCorrectionAlphaFilter");
      interpolationAlphaFilterAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationAlphaFilterAlphaValue");
      interpolationAlphaFilterBreakFrequency = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationAlphaFilterBreakFrequency");

      confidenceFactor = (DoubleYoVariable) registry.getVariable(nameSpace, "PelvisErrorCorrectionConfidenceFactor");

      externalPelvisPositionX = (DoubleYoVariable) registry.getVariable(nameSpace, "newExternalPelvis_positionX");
      externalPelvisPositionY = (DoubleYoVariable) registry.getVariable(nameSpace, "newExternalPelvis_positionY");
      externalPelvisPositionZ = (DoubleYoVariable) registry.getVariable(nameSpace, "newExternalPelvis_positionZ");
      externalPelvisYaw = (DoubleYoVariable) registry.getVariable(nameSpace, "newExternalPelvis_yaw");
      externalPelvisPitch = (DoubleYoVariable) registry.getVariable(nameSpace, "newExternalPelvis_pitch");
      externalPelvisRoll = (DoubleYoVariable) registry.getVariable(nameSpace, "newExternalPelvis_roll");

      totalErrorPelvisPositionX = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorPelvis_positionX");
      totalErrorPelvisPositionY = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorPelvis_positionY");
      totalErrorPelvisPositionZ = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorPelvis_positionZ");
      totalErrorPelvisYaw = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorPelvis_yaw");
      totalErrorPelvisPitch = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorPelvis_pitch");
      totalErrorPelvisRoll = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorPelvis_roll");

      interpolatedPelvisErrorPositionX = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedPelvisError_positionX");
      interpolatedPelvisErrorPositionY = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedPelvisError_positionY");
      interpolatedPelvisErrorPositionZ = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedPelvisError_positionZ");
      interpolatedPelvisErrorYaw = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedPelvisError_yaw");
      interpolatedPelvisErrorPitch = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedPelvisError_pitch");
      interpolatedPelvisErrorRoll = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedPelvisError_roll");

      previousInterpolatedPelvisErrorPositionX = (DoubleYoVariable) registry.getVariable(nameSpace, "previousInterpolatedPelvisError_positionX");
      previousInterpolatedPelvisErrorPositionY = (DoubleYoVariable) registry.getVariable(nameSpace, "previousInterpolatedPelvisError_positionY");
      previousInterpolatedPelvisErrorPositionZ = (DoubleYoVariable) registry.getVariable(nameSpace, "previousInterpolatedPelvisError_positionZ");
      previousInterpolatedPelvisErrorYaw = (DoubleYoVariable) registry.getVariable(nameSpace, "previousInterpolatedPelvisError_yaw");
      previousInterpolatedPelvisErrorPitch = (DoubleYoVariable) registry.getVariable(nameSpace, "previousInterpolatedPelvisError_pitch");
      previousInterpolatedPelvisErrorRoll = (DoubleYoVariable) registry.getVariable(nameSpace, "previousInterpolatedPelvisError_roll");

      seNonProcessedPelvisPositionX = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_positionX");
      seNonProcessedPelvisPositionY = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_positionY");
      seNonProcessedPelvisPositionZ = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_positionZ");
      seNonProcessedPelvisYaw = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_yaw");
      seNonProcessedPelvisPitch = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_pitch");
      seNonProcessedPelvisRoll = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_roll");

      seNonProcessedPelvisTimeStamp = (LongYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_timestamp");

      correctedPelvisPositionX = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_positionX");
      correctedPelvisPositionY = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_positionY");
      correctedPelvisPositionZ = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_positionZ");
      correctedPelvisYaw = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_yaw");
      correctedPelvisPitch = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_pitch");
      correctedPelvisRoll = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_roll");

      seNonProcessedPelvisQuaternionX = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_quaternionQx");
      seNonProcessedPelvisQuaternionY = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_quaternionQy");
      seNonProcessedPelvisQuaternionZ = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_quaternionQz");
      seNonProcessedPelvisQuaternionS = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_quaternionQs");

      maxVelocityClip = (DoubleYoVariable) registry.getVariable(nameSpace, "maxVelocityClip");
      clippedAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "clippedAlphaValue");
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

   @Test
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

         double xError = Math.abs(correctedPelvisPositionX.getDoubleValue() - error.getX());
         double yError = Math.abs(correctedPelvisPositionY.getDoubleValue() - error.getY());
         double zError = Math.abs(correctedPelvisPositionZ.getDoubleValue() - error.getZ());
         double yawError = Math.abs(correctedPelvisYaw.getDoubleValue() - error.getW());

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

         double xError = Math.abs(correctedPelvisPositionX.getDoubleValue() - error.getX());
         double yError = Math.abs(correctedPelvisPositionY.getDoubleValue() - error.getY());
         double zError = Math.abs(correctedPelvisPositionZ.getDoubleValue() - error.getZ());
         double yawError = Math.abs(correctedPelvisYaw.getDoubleValue() - error.getW());

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

         double xError = Math.abs(correctedPelvisPositionX.getDoubleValue() - error.getX());
         double yError = Math.abs(correctedPelvisPositionY.getDoubleValue() - error.getY());
         double zError = Math.abs(correctedPelvisPositionZ.getDoubleValue() - error.getZ());
         double yawError = Math.abs(correctedPelvisYaw.getDoubleValue() - error.getW());

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

   private class ExternalPelvisPoseCreator implements ExternalPelvisPoseSubscriberInterface
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
      public void consumeObject(StampedPosePacket object)
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
