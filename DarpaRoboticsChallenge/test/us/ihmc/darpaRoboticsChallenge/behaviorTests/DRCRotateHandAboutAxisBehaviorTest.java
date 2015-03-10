package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.LinkedHashMap;

import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.RotateHandAboutAxisBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.io.printing.SysoutTool;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCRotateHandAboutAxisBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      GlobalTimer.clearTimers();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCChestOrientationBehaviorTest.class + " after class.");
   }

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   private ArmJointName[] armJointNames;
   private int numberOfArmJoints;
   private LinkedHashMap<ArmJointName, Integer> armJointIndices = new LinkedHashMap<ArmJointName, Integer>();

   private static final boolean DEBUG = false;

   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      KryoPacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
            PacketDestination.CONTROLLER.ordinal(), "DRCControllerCommunicator");
      KryoPacketCommunicator networkObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
            PacketDestination.NETWORK_PROCESSOR.ordinal(), "MockNetworkProcessorCommunicator");

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(new DRCDemo01NavigationEnvironment(), networkObjectCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel(), controllerCommunicator);

      this.armJointNames = drcBehaviorTestHelper.getSDFFullRobotModel().getRobotSpecificJointNames().getArmJointNames();
      this.numberOfArmJoints = armJointNames.length;
      for (int i = 0; i < numberOfArmJoints; i++)
      {
         armJointIndices.put(armJointNames[i], i);
      }
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testRotateInJointSpaceAndUnRotateInTaskSpace() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      SysoutTool.println("Initializing Simulation", DEBUG);
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 2.0;
      ReferenceFrame handFrame = drcBehaviorTestHelper.getSDFFullRobotModel().getHandControlFrame(robotSide);
      ArmJointName wristPitchJointName = ArmJointName.WRIST_PITCH;
      OneDoFJoint wristPitchOneDoFJoint = drcBehaviorTestHelper.getSDFFullRobotModel().getArmJoint(robotSide, wristPitchJointName);

      SysoutTool.println("Determining which Axis (X,Y,or Z) in " + handFrame + " is aligned with: " + wristPitchJointName, DEBUG);
      FramePose initialHandPose = getCurrentHandPose(robotSide);
      Axis handFrameAxisAlignedWithWristPitchJoint = getReferenceFrameAxisThatIsAlignedWithParentArmJoint(robotSide, handFrame, wristPitchJointName);
      FramePose handPoseAfterJointSpaceRotation = getCurrentHandPose(robotSide);
      SysoutTool.println("Axis aligned with " + wristPitchJointName + " : " + handFrameAxisAlignedWithWristPitchJoint, DEBUG);

      double jointSpaceRotation = initialHandPose.getOrientationDistance(handPoseAfterJointSpaceRotation);
      assertEquals("", Math.toRadians(90.0), Math.abs(jointSpaceRotation), Math.toRadians(1.0));

      double turnAngleRad = Math.toRadians(90.0);
      double q_wristPitchInitial = getCurrentArmPose(robotSide)[armJointIndices.get(wristPitchJointName)];
      double q_wristDesired = -getJointMotionDirectionThatDoesNotExceedJointLimits(wristPitchOneDoFJoint, q_wristPitchInitial, turnAngleRad);

      boolean useWholeBodyInverseKinematics = false;
      final RotateHandAboutAxisBehavior rotateHandBehavior = createNewRotateHandBehavior(useWholeBodyInverseKinematics);
      rotateHandBehavior.initialize();
      rotateHandBehavior.setInput(robotSide, handPoseAfterJointSpaceRotation, handFrameAxisAlignedWithWristPitchJoint, handFrame.getTransformToWorldFrame(),
            q_wristDesired, trajectoryTime);

      success = drcBehaviorTestHelper.executeBehaviorUntilDone(rotateHandBehavior);
      FramePose handPoseAfterTaskSpaceUnRotation = getCurrentHandPose(robotSide);

      success = success & rotateHandBehavior.isDone();
      assertTrue(success);
      assertPosesAreWithinThresholds(initialHandPose, handPoseAfterTaskSpaceUnRotation, 0.05, Math.toRadians(25.0));  //TODO: Looks decent in sim, but need to figure out why orientation threshold is so large.

      BambooTools.reportTestFinishedMessage();
   }
   
   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testRotateInJointSpaceAndUnRotateInTaskSpaceUsingWholeBodyInverseKinematics() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      SysoutTool.println("Initializing Simulation", DEBUG);
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 2.0;
      ReferenceFrame handFrame = drcBehaviorTestHelper.getSDFFullRobotModel().getHandControlFrame(robotSide);
      ArmJointName wristPitchJointName = ArmJointName.WRIST_PITCH;
      OneDoFJoint wristPitchOneDoFJoint = drcBehaviorTestHelper.getSDFFullRobotModel().getArmJoint(robotSide, wristPitchJointName);

      SysoutTool.println("Determining which Axis (X,Y,or Z) in " + handFrame + " is aligned with: " + wristPitchJointName, DEBUG);
      FramePose initialHandPose = getCurrentHandPose(robotSide);
      Axis handFrameAxisAlignedWithWristPitchJoint = getReferenceFrameAxisThatIsAlignedWithParentArmJoint(robotSide, handFrame, wristPitchJointName);
      FramePose handPoseAfterJointSpaceRotation = getCurrentHandPose(robotSide);
      SysoutTool.println("Axis aligned with " + wristPitchJointName + " : " + handFrameAxisAlignedWithWristPitchJoint, DEBUG);

      double jointSpaceRotation = initialHandPose.getOrientationDistance(handPoseAfterJointSpaceRotation);
      assertEquals("", Math.toRadians(90.0), Math.abs(jointSpaceRotation), Math.toRadians(1.0));

      double turnAngleRad = Math.toRadians(90.0);
      double q_wristPitchInitial = getCurrentArmPose(robotSide)[armJointIndices.get(wristPitchJointName)];
      double q_wristDesired = -getJointMotionDirectionThatDoesNotExceedJointLimits(wristPitchOneDoFJoint, q_wristPitchInitial, turnAngleRad);

      boolean useWholeBodyInverseKinematics = true;
      final RotateHandAboutAxisBehavior rotateHandBehavior = createNewRotateHandBehavior(useWholeBodyInverseKinematics);
      rotateHandBehavior.initialize();
      rotateHandBehavior.setInput(robotSide, handPoseAfterJointSpaceRotation, handFrameAxisAlignedWithWristPitchJoint, handFrame.getTransformToWorldFrame(),
            q_wristDesired, trajectoryTime);

      success = drcBehaviorTestHelper.executeBehaviorUntilDone(rotateHandBehavior);
      FramePose handPoseAfterTaskSpaceUnRotation = getCurrentHandPose(robotSide);

      success = success & rotateHandBehavior.isDone();
      assertTrue(success);
      assertPosesAreWithinThresholds(initialHandPose, handPoseAfterTaskSpaceUnRotation, 0.05, Math.toRadians(25.0));  //TODO: Looks decent in sim, but need to figure out why orientation threshold is so large.

      BambooTools.reportTestFinishedMessage();
   }

   private void printArmJointAngles(double[] armJointAngles)
   {
      for (int i = 0; i < numberOfArmJoints; i++)
      {
         SysoutTool.println("desired q_" + armJointNames[i] + " : " + armJointAngles[i]);
      }
   }

   private RotateHandAboutAxisBehavior createNewRotateHandBehavior(boolean useWholeBodyInverseKinematics)
   {
      BehaviorCommunicationBridge communicationBridge = drcBehaviorTestHelper.getBehaviorCommunicationBridge();
      SDFFullRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      ReferenceFrames referenceFrames = drcBehaviorTestHelper.getReferenceFrames();
      DoubleYoVariable yoTime = drcBehaviorTestHelper.getYoTime();

      RotateHandAboutAxisBehavior ret = new RotateHandAboutAxisBehavior(communicationBridge, fullRobotModel, referenceFrames, getRobotModel(), yoTime, useWholeBodyInverseKinematics);
      return ret;
   }

   private FramePose getCurrentHandPose(RobotSide robotSideToTest)
   {
      FramePose ret = new FramePose();
      drcBehaviorTestHelper.updateRobotModel();
      ret.setToZero(drcBehaviorTestHelper.getSDFFullRobotModel().getHandControlFrame(robotSideToTest));
      ret.changeFrame(ReferenceFrame.getWorldFrame());
      return ret;
   }

   private FramePose getRobotPose(ReferenceFrames referenceFrames)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FramePose ret = new FramePose();

      drcBehaviorTestHelper.updateRobotModel();
      ReferenceFrame midFeetFrame = referenceFrames.getMidFeetZUpFrame();

      ret.setToZero(midFeetFrame);
      ret.changeFrame(worldFrame);

      return ret;
   }

   private Axis getReferenceFrameAxisThatIsAlignedWithParentArmJoint(RobotSide robotSide, ReferenceFrame referenceFrame, ArmJointName parentOfReferenceFrame)
         throws SimulationExceededMaximumTimeException
   {
      drcBehaviorTestHelper.updateRobotModel();
      double turnAngleRad = Math.toRadians(90.0);
      double trajectoryTime = 1.0;

      OneDoFJoint wristPitchOneDoFJoint = drcBehaviorTestHelper.getSDFFullRobotModel().getArmJoint(robotSide, parentOfReferenceFrame);
      double[] initialJointAngles = getCurrentArmPose(robotSide);
      if (DEBUG)
         printArmJointAngles(initialJointAngles);

      double q_wristPitchInitial = initialJointAngles[armJointIndices.get(parentOfReferenceFrame)];
      double q_wristDesired = getJointMotionDirectionThatDoesNotExceedJointLimits(wristPitchOneDoFJoint, q_wristPitchInitial, turnAngleRad);

      ArrayList<FrameVector> handFrameXYZAxes = new ArrayList<FrameVector>();
      handFrameXYZAxes.add(new FrameVector(referenceFrame, 1.0, 0.0, 0.0));
      handFrameXYZAxes.add(new FrameVector(referenceFrame, 0.0, 1.0, 0.0));
      handFrameXYZAxes.add(new FrameVector(referenceFrame, 0.0, 0.0, 1.0));

      double[] desiredJointAngles = initialJointAngles;
      setSingleJoint(desiredJointAngles, robotSide, parentOfReferenceFrame, q_wristDesired, true);

      if (q_wristDesired != desiredJointAngles[armJointIndices.get(parentOfReferenceFrame)])
         throw new RuntimeException("Desired joint angles array has not been properly set.  Desired wrist pitch angle (" + q_wristDesired
               + ") does not equal that specified in the array (" + desiredJointAngles[armJointIndices.get(parentOfReferenceFrame)]);

      if (DEBUG)
         printArmJointAngles(desiredJointAngles);

      ArrayList<Vector3d> initialHandFrameAxesInWorld = new ArrayList<Vector3d>();
      for (FrameVector framePoint : handFrameXYZAxes)
      {
         framePoint.changeFrame(ReferenceFrame.getWorldFrame());
         initialHandFrameAxesInWorld.add(framePoint.getVectorCopy());
         framePoint.changeFrame(referenceFrame);
      }

      final HandPoseBehavior rotateWristPitchBehavior = new HandPoseBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            drcBehaviorTestHelper.getYoTime());
      rotateWristPitchBehavior.initialize();
      HandPosePacket handPosePacket = new HandPosePacket(robotSide, trajectoryTime, desiredJointAngles);
      rotateWristPitchBehavior.setInput(handPosePacket);

      boolean success = drcBehaviorTestHelper.executeBehaviorUntilDone(rotateWristPitchBehavior);
      assertTrue(success);

      ArrayList<Vector3d> finalHandFrameAxesInWorld = new ArrayList<Vector3d>();
      double maxDotProduct = 0.0;
      Axis axisAlignedWithWristPitchJoint = null;

      int i = 0;
      System.out.println("\n");
      for (FrameVector framePoint : handFrameXYZAxes)
      {
         framePoint.changeFrame(ReferenceFrame.getWorldFrame());
         finalHandFrameAxesInWorld.add(framePoint.getVectorCopy());
         framePoint.changeFrame(referenceFrame);

         Axis currentAxis = Axis.values()[i];

         double dotProduct = initialHandFrameAxesInWorld.get(i).dot(finalHandFrameAxesInWorld.get(i));
         if (DEBUG)
            SysoutTool.println(currentAxis + "dotProduct : " + dotProduct);

         if (Math.abs(dotProduct) > maxDotProduct)
         {
            maxDotProduct = Math.abs(dotProduct);
            axisAlignedWithWristPitchJoint = currentAxis;
         }

         i++;
      }

      return axisAlignedWithWristPitchJoint;
   }

   private double getJointMotionDirectionThatDoesNotExceedJointLimits(OneDoFJoint wristPitchOneDoFJoint, double q_wristPitchInitial, double turnAngleRad)
   {

      double q_wristPitchInitialProximityToLowerLimit = Math.abs(q_wristPitchInitial - wristPitchOneDoFJoint.getJointLimitLower());
      double q_wristPitchInitialProximityToUpperLimit = Math.abs(q_wristPitchInitial - wristPitchOneDoFJoint.getJointLimitUpper());

      double q_wristDesired = q_wristPitchInitial + turnAngleRad;

      if (q_wristPitchInitialProximityToLowerLimit < Math.abs(turnAngleRad))
      {
         q_wristDesired = q_wristPitchInitial + Math.abs(turnAngleRad);
      }
      else if (q_wristPitchInitialProximityToUpperLimit < Math.abs(turnAngleRad))
      {
         q_wristDesired = q_wristPitchInitial - Math.abs(turnAngleRad);
      }
      return q_wristDesired;
   }

   private void setSingleJoint(double[] armPoseToPack, RobotSide robotSide, ArmJointName armJointName, double desiredJointAngle,
         boolean clipDesiredQToJointLimits)
   {
      if (clipDesiredQToJointLimits)
         desiredJointAngle = clipDesiredToJointLimits(robotSide, armJointName, desiredJointAngle);

      armPoseToPack[armJointIndices.get(armJointName)] = desiredJointAngle;
   }

   private double clipDesiredToJointLimits(RobotSide robotSide, ArmJointName armJointName, double desiredJointAngle)
   {
      FullRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      double q;
      double qMin = fullRobotModel.getArmJoint(robotSide, armJointName).getJointLimitLower();
      double qMax = fullRobotModel.getArmJoint(robotSide, armJointName).getJointLimitUpper();

      if (qMin > qMax)
      {
         double temp = qMax;
         qMax = qMin;
         qMin = temp;
      }

      q = MathTools.clipToMinMax(desiredJointAngle, qMin, qMax);
      return q;
   }

   private double[] getCurrentArmPose(RobotSide robotSide)
   {
      double[] armPose = new double[numberOfArmJoints];

      for (int jointNum = 0; jointNum < numberOfArmJoints; jointNum++)
      {
         ArmJointName jointName = armJointNames[jointNum];
         double currentAngle = drcBehaviorTestHelper.getSDFFullRobotModel().getArmJoint(robotSide, jointName).getQ();
         armPose[jointNum] = currentAngle;
      }

      return armPose;
   }

   private void assertPosesAreWithinThresholds(FramePose desiredPose, FramePose actualPose, double positionThreshold, double orientationThreshold)
   {
      double positionDistance = desiredPose.getPositionDistance(actualPose);
      double orientationDistance = desiredPose.getOrientationDistance(actualPose);

      if (DEBUG)
      {
         System.out.println("positionDistance=" + positionDistance);
         System.out.println("orientationDistance=" + orientationDistance);
      }
      assertEquals("Pose position error :" + positionDistance + " exceeds threshold: " + positionThreshold, 0.0, positionDistance, positionThreshold);
      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + orientationThreshold, 0.0, orientationDistance,
            orientationThreshold);
   }

}
