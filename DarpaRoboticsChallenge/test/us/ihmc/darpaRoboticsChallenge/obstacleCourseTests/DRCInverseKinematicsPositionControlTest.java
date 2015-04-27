package us.ihmc.darpaRoboticsChallenge.obstacleCourseTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.JointPositionControllerFactory;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.dataProcessors.RobotAllJointsDataChecker;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.controllers.GainCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public abstract class DRCInverseKinematicsPositionControlTest implements MultiRobotTestInterface
{

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
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   public static final double POSITION_THRESHOLD = 0.01;
   public static final double ORIENTATION_THRESHOLD = 0.03;
   public static final double JOINT_POSITION_THRESHOLD = 0.05;
   private static final boolean DEBUG = false;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private ArmJointName[] armJointNames;
   private int numberOfArmJoints;
   private LinkedHashMap<ArmJointName, Integer> armJointIndices = new LinkedHashMap<ArmJointName, Integer>();
   
   @Before
   public void setUp()
   { 
      drcSimulationTestHelper = new DRCSimulationTestHelper(new DRCDemo01NavigationEnvironment(), "PositionControlTest", "", DRCObstacleCourseStartingLocation.DEFAULT,
            simulationTestingParameters, getRobotModel(), null, new JointPositionControllerFactory(true), null);

      armJointNames = drcSimulationTestHelper.getSDFFullRobotModel().getRobotSpecificJointNames().getArmJointNames();
      numberOfArmJoints = armJointNames.length;

      for (int i = 0; i < numberOfArmJoints; i++)
      {
         armJointIndices.put(armJointNames[i], i);
      }
      
      SDFRobot robot = drcSimulationTestHelper.getRobot();
      
      double robotFloatingHeight = 0.1;
      LockPelvisController controller = new LockPelvisController(robot, drcSimulationTestHelper.getSimulationConstructionSet(), drcSimulationTestHelper.getSDFFullRobotModel(), robotFloatingHeight);
      robot.setController(controller);
      controller.initialize();
   }
   
   
	@EstimatedDuration(duration = 105.6)
	@Test(timeout = 316687)
   public void testJointSpaceHandPose() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      
      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 2.0;
      double[] desiredJointAngles = createRandomArmPose(robotSide);
      
      HandPosePacket jointSpacePacket = new HandPosePacket(robotSide, trajectoryTime, desiredJointAngles);
      drcSimulationTestHelper.send(jointSpacePacket);
      
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 1.0);
      
      assertTrue(success);
      assertCurrentHandPoseIsWithinThresholds(robotSide, desiredJointAngles);
      assertArmJointsMovedSmoothlyWithinKinematicAndDynamicLimits();
      
      BambooTools.reportTestFinishedMessage();
   }
	
   @EstimatedDuration(duration = 105.6)
   @Test(timeout = 316687)
   public void testTaskSpaceHandPose() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 2.0;
      double[] initialJointAngles = getCurrentArmPose(robotSide);
      double[] desiredJointAngles = createRandomArmPose(robotSide);

      drcSimulationTestHelper.send(new HandPosePacket(robotSide, trajectoryTime, desiredJointAngles));
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 1.0);
      assertTrue(success);
      assertCurrentHandPoseIsWithinThresholds(robotSide, desiredJointAngles);
      
      FramePose reachableHandPose = getCurrentHandPose(robotSide);
      reachableHandPose.changeFrame(ReferenceFrame.getWorldFrame());
      RigidBodyTransform reachableHandPoseTransformToWorld = new RigidBodyTransform();
      reachableHandPose.getPose(reachableHandPoseTransformToWorld);
      
      drcSimulationTestHelper.send(new HandPosePacket(robotSide, trajectoryTime, initialJointAngles));
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 1.0);
      assertTrue(success);
      assertCurrentHandPoseIsWithinThresholds(robotSide, initialJointAngles);

      drcSimulationTestHelper.send(PacketControllerTools.createHandPosePacket(Frame.WORLD, reachableHandPoseTransformToWorld, robotSide, trajectoryTime));
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 1.0);
      
      assertTrue(success);
      assertCurrentHandPoseIsWithinThresholds(robotSide, reachableHandPose);
      assertArmJointsMovedSmoothlyWithinKinematicAndDynamicLimits();

      BambooTools.reportTestFinishedMessage();
   }
   
   private FramePose getCurrentHandPose(RobotSide robotSideToTest)
   {
      FramePose ret = new FramePose();
      ret.setToZero(drcSimulationTestHelper.getControllerFullRobotModel().getHandControlFrame(robotSideToTest));
      ret.changeFrame(ReferenceFrame.getWorldFrame());
      return ret;
   }
	
   private double[] getCurrentArmPose(RobotSide robotSide)
   {      
      double[] armPose = new double[numberOfArmJoints];

      for (int jointNum = 0; jointNum < numberOfArmJoints; jointNum++)
      {
         ArmJointName jointName = armJointNames[jointNum];
         double currentAngle = drcSimulationTestHelper.getControllerFullRobotModel().getArmJoint(robotSide, jointName).getQ();
         armPose[jointNum] = currentAngle;
      }

      return armPose;
   }

   private double[] createRandomArmPose(RobotSide robotSide)
   {
      double[] armPose = new double[numberOfArmJoints];

      for (int jointNum = 0; jointNum < numberOfArmJoints; jointNum++)
      {
         double qDesired = clipDesiredJointQToJointLimits(robotSide, armJointNames[jointNum], RandomTools.generateRandomDouble(new Random(), 1.5));
         armPose[jointNum] = qDesired;
      }

      return armPose;
   }

   private double clipDesiredJointQToJointLimits(RobotSide robotSide, ArmJointName armJointName, double desiredJointAngle)
   {
      FullRobotModel fullRobotModel = drcSimulationTestHelper.getSDFFullRobotModel();

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
   
   private void assertCurrentHandPoseIsWithinThresholds(RobotSide robotSide, double[] desiredArmPose)
   {
      double[] currentArmPose = getCurrentArmPose(robotSide);
      assertPosesAreWithinThresholds(desiredArmPose, currentArmPose, robotSide);
   }

   private void assertPosesAreWithinThresholds(double[] desiredArmPose, double[] actualArmPose, RobotSide robotSide)
   {
      for (int i = 0; i < numberOfArmJoints; i++)
      {
         ArmJointName armJointName = armJointNames[i];

         double q_desired = desiredArmPose[i];
         double q_actual = actualArmPose[i];
         double error = Math.abs(q_actual - q_desired);

         if (DEBUG)
         {
            PrintTools.debug(this, armJointName + " qDesired = " + q_desired + ".  qActual = " + q_actual + ".");
         }
         assertEquals(armJointName + " position error (" + Math.toDegrees(error) + " degrees) exceeds threshold of " + Math.toDegrees(JOINT_POSITION_THRESHOLD)
               + " degrees.", q_desired, q_actual, JOINT_POSITION_THRESHOLD);
      }
   }
   
   private void assertCurrentHandPoseIsWithinThresholds(RobotSide robotSide, FramePose desiredPose)
   {
      FramePose currentPose = getCurrentHandPose(robotSide);
      assertPosesAreWithinThresholds(desiredPose, currentPose);
   }

   private void assertPosesAreWithinThresholds(FramePose desiredPose, FramePose actualPose)
   {
      assertPosesAreWithinThresholds(desiredPose, actualPose, POSITION_THRESHOLD, ORIENTATION_THRESHOLD);
   }

   private void assertPosesAreWithinThresholds(FramePose desiredPose, FramePose actualPose, double positionThreshold, double orientationThreshold)
   {
      double positionDistance = desiredPose.getPositionDistance(actualPose);
      double orientationDistance = desiredPose.getOrientationDistance(actualPose);

      if (DEBUG)
      {
         System.out.println("testSimpleHandPoseMove: positionDistance=" + positionDistance);
         System.out.println("testSimpleHandPoseMove: orientationDistance=" + orientationDistance);
      }

      assertEquals("Pose position error :" + positionDistance + " exceeds threshold: " + positionThreshold, 0.0, positionDistance, positionThreshold);
      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + orientationThreshold, 0.0, orientationDistance,
            orientationThreshold);
   }
   
   private final double maximumJointVelocity = 1.5;
   private final double maximumJointAcceleration = 15.0;
   
   private void assertArmJointsMovedSmoothlyWithinKinematicAndDynamicLimits()
   {
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      Robot robot = drcSimulationTestHelper.getRobot();
      
      ArrayList<OneDegreeOfFreedomJoint> allJoints = new ArrayList<>();
      ArrayList<OneDegreeOfFreedomJoint> jointsToCheck = new ArrayList<>();

      robot.getAllOneDegreeOfFreedomJoints(allJoints);

      for (OneDegreeOfFreedomJoint joint : allJoints)
      {
         if (!joint.getName().contains("leg") && !joint.getName().contains("back") && !joint.getName().contains("ankle") && !joint.getName().contains("neck")
               && !joint.getName().contains("hokuyo"))
            jointsToCheck.add(joint);
      }

      RobotAllJointsDataChecker checker = new RobotAllJointsDataChecker(scs, robot, jointsToCheck);

      checker.setMaximumDerivativeForAllJoints(10.2 * maximumJointVelocity);
      checker.setMaximumSecondDerivativeForAllJoints(1.2 * maximumJointAcceleration);
    
      checker.cropFirstPoint();
      
      scs.applyDataProcessingFunction(checker);
      
      assertFalse(checker.getDerivativeCompError(), checker.hasDerivativeComparisonErrorOccurredAnyJoint());
      assertFalse(checker.getMaxDerivativeExceededError(), checker.hasMaxDerivativeExeededAnyJoint());
//      assertFalse(checker.getMaxSecondDerivativeExceededError(), checker.hasMaxSecondDerivativeExeededAnyJoint());
      assertFalse(checker.getMaxValueExceededError(), checker.hasMaxValueExeededAnyJoint());
      assertFalse(checker.getMinValueExceededError(), checker.hasMinValueExeededAnyJoint());
   }
   
   
   private class LockPelvisController implements RobotController
   {
      private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

      private final ArrayList<ExternalForcePoint> externalForcePoints = new ArrayList<>();
      private final ArrayList<Vector3d> efp_offsetFromRootJoint = new ArrayList<>();
      private final double dx = 0.5, dy = 0.5, dz = 0.0*1.0;

      private final ArrayList<Vector3d> initialPositions = new ArrayList<>();

      private final DoubleYoVariable holdPelvisKp = new DoubleYoVariable("holdPelvisKp", registry);
      private final DoubleYoVariable holdPelvisKv = new DoubleYoVariable("holdPelvisKv", registry);
      private final DoubleYoVariable desiredHeight = new DoubleYoVariable("desiredHeight", registry);
      private final double robotMass, robotWeight;

      private final SDFRobot robot;

      private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      private final ArrayList<YoGraphicPosition> efp_positionViz = new ArrayList<>();

      public LockPelvisController(SDFRobot robot, SimulationConstructionSet scs, SDFFullRobotModel sdfFullRobotModel, double desiredHeight)
      {
         this.robot = robot;
         robotMass = robot.computeCenterOfMass(new Point3d());
         robotWeight = robotMass * Math.abs(robot.getGravityZ());
         this.desiredHeight.set(desiredHeight);

         Joint jointToAddExternalForcePoints;
         try
         {
            //            String lastSpineJointName = sdfFullRobotModel.getChest().getParentJoint().getName();
            jointToAddExternalForcePoints = robot.getJoint(sdfFullRobotModel.getPelvis().getParentJoint().getName());
         }
         catch (NullPointerException e)
         {
            System.err.println("No chest or spine found. Stack trace:");
            e.printStackTrace();

            jointToAddExternalForcePoints = robot.getPelvisJoint();
         }

         holdPelvisKp.set(5000.0);
         holdPelvisKv.set(GainCalculator.computeDampingForSecondOrderSystem(robotMass, holdPelvisKp.getDoubleValue(), 0.6));

         efp_offsetFromRootJoint.add(new Vector3d(dx, dy, dz));
         efp_offsetFromRootJoint.add(new Vector3d(dx, -dy, dz));
         efp_offsetFromRootJoint.add(new Vector3d(-dx, dy, dz));
         efp_offsetFromRootJoint.add(new Vector3d(-dx, -dy, dz));

         for (int i = 0; i < efp_offsetFromRootJoint.size(); i++)
         {
            initialPositions.add(new Vector3d());

            String linkName = jointToAddExternalForcePoints.getLink().getName();
            ExternalForcePoint efp = new ExternalForcePoint("efp_" + linkName + "_" + String.valueOf(i) + "_", efp_offsetFromRootJoint.get(i),
                  robot.getRobotsYoVariableRegistry());
            externalForcePoints.add(efp);
            jointToAddExternalForcePoints.addExternalForcePoint(efp);

            efp_positionViz.add(new YoGraphicPosition(efp.getName(), efp.getYoPosition(), 0.05, YoAppearance.Red()));
         }

         yoGraphicsListRegistry.registerYoGraphics("EFP", efp_positionViz);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      }

      @Override
      public void initialize()
      {
         robot.update();
         for (int i = 0; i < efp_offsetFromRootJoint.size(); i++)
         {
            externalForcePoints.get(i).getYoPosition().get(initialPositions.get(i));
            desiredHeight.add(initialPositions.get(i).z / initialPositions.size());
            efp_positionViz.get(i).update();
         }

         for (int i = 0; i < efp_offsetFromRootJoint.size(); i++)
            initialPositions.get(i).setZ(desiredHeight.getDoubleValue());

         doControl();
      }

      private final Vector3d proportionalTerm = new Vector3d();
      private final Vector3d derivativeTerm = new Vector3d();
      private final Vector3d pdControlOutput = new Vector3d();

      @Override
      public void doControl()
      {
         for (int i = 0; i < efp_offsetFromRootJoint.size(); i++)
         {
            initialPositions.get(i).setZ(desiredHeight.getDoubleValue());

            ExternalForcePoint efp = externalForcePoints.get(i);
            efp.getYoPosition().get(proportionalTerm);
            proportionalTerm.sub(initialPositions.get(i));
            proportionalTerm.scale(-holdPelvisKp.getDoubleValue());
//            proportionalTerm.setZ(Math.max(proportionalTerm.getZ(), 0.0));

            efp.getYoVelocity().get(derivativeTerm);
            derivativeTerm.scale(-holdPelvisKv.getDoubleValue());

            pdControlOutput.add(proportionalTerm, derivativeTerm);

            efp.setForce(pdControlOutput);
            efp.getYoForce().getYoZ().add(robotWeight / efp_offsetFromRootJoint.size());

            efp_positionViz.get(i).update();
         }
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return registry.getName();
      }

      @Override
      public String getDescription()
      {
         return getName();
      }
   }

}
