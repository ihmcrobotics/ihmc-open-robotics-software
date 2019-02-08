package us.ihmc.avatar.controllerAPI;

import java.util.Map;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.AvatarCommonAsserts;
import us.ihmc.avatar.testTools.AvatarRandomTestMessages;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

public abstract class EndToEndGoHomeMessageTest implements MultiRobotTestInterface
{
   private static final Random random = new Random(2943L);

   private SimulationTestingParameters simulationTestingParameters;
   private DRCSimulationTestHelper testHelper;

   public void testGoHomeArms() throws SimulationExceededMaximumTimeException
   {
      createSimulation();

      double trajectoryTime = 0.5;
      FullHumanoidRobotModel robot = testHelper.getControllerFullRobotModel();
      SimulationConstructionSet scs = testHelper.getSimulationConstructionSet();
      double[] leftHome = getArmHome(getRobotModel(), RobotSide.LEFT, robot);
      double[] rightHome = getArmHome(getRobotModel(), RobotSide.RIGHT, robot);

      testHelper.publishToController(AvatarRandomTestMessages.nextArmTrajectoryMessage(random, trajectoryTime, RobotSide.LEFT, robot));
      testHelper.publishToController(AvatarRandomTestMessages.nextArmTrajectoryMessage(random, trajectoryTime, RobotSide.RIGHT, robot));

      Assert.assertTrue(testHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 0.25));

      testHelper.publishToController(HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.ARM, RobotSide.LEFT, trajectoryTime));
      testHelper.publishToController(HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.ARM, RobotSide.RIGHT, trajectoryTime));

      Assert.assertTrue(testHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 0.25));

      AvatarCommonAsserts.assertDesiredArmPositions(leftHome, RobotSide.LEFT, robot, scs, 1.0e-10);
      AvatarCommonAsserts.assertDesiredArmPositions(rightHome, RobotSide.RIGHT, robot, scs, 1.0e-10);
      AvatarCommonAsserts.assertDesiredArmVelocitiesZero(RobotSide.LEFT, robot, scs, 1.0e-10);
      AvatarCommonAsserts.assertDesiredArmVelocitiesZero(RobotSide.RIGHT, robot, scs, 1.0e-10);

      Assert.assertTrue(testHelper.simulateAndBlockAndCatchExceptions(2.0));

      AvatarCommonAsserts.assertArmPositions(leftHome, RobotSide.LEFT, robot, Math.toRadians(15.0));
      AvatarCommonAsserts.assertArmPositions(rightHome, RobotSide.RIGHT, robot, Math.toRadians(15.0));
   }

   public void testGoHomeChest() throws SimulationExceededMaximumTimeException
   {
      createSimulation();

      double trajectoryTime = 0.5;
      MovingReferenceFrame pelvisZUpFrame = testHelper.getReferenceFrames().getPelvisZUpFrame();
      FullHumanoidRobotModel robot = testHelper.getControllerFullRobotModel();
      SimulationConstructionSet scs = testHelper.getSimulationConstructionSet();
      FrameQuaternion chestHome = new FrameQuaternion(pelvisZUpFrame, getChestHome(getRobotModel(), robot).getOrientation());

      testHelper.publishToController(AvatarRandomTestMessages.nextChestTrajectoryMessage(random, trajectoryTime, pelvisZUpFrame, robot));

      Assert.assertTrue(testHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 0.25));

      testHelper.publishToController(HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.CHEST, trajectoryTime));

      Assert.assertTrue(testHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 0.25));

      AvatarCommonAsserts.assertDesiredChestOrientation(chestHome, robot, scs, 1.0e-6);
      AvatarCommonAsserts.assertDesiredChestAngularVelocityZero(robot, scs, 1.0e-6);

      Assert.assertTrue(testHelper.simulateAndBlockAndCatchExceptions(2.0));

      AvatarCommonAsserts.assertChestOrientation(chestHome, robot, Math.toRadians(1.0));
   }

   public void testGoHomePelvis() throws SimulationExceededMaximumTimeException
   {
      createSimulation();

      double trajectoryTime = 1.0;
      FullHumanoidRobotModel robot = testHelper.getControllerFullRobotModel();
      SimulationConstructionSet scs = testHelper.getSimulationConstructionSet();

      testHelper.publishToController(AvatarRandomTestMessages.nextPelvisTrajectoryMessage(random, trajectoryTime, robot, 0.1, Math.toRadians(30.0)));

      Assert.assertTrue(testHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 0.5));

      testHelper.publishToController(HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.PELVIS, trajectoryTime));

      Assert.assertTrue(testHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 1.0));

      FrameQuaternion pelvisHomeOrientation = new FrameQuaternion(testHelper.getReferenceFrames().getMidFeetZUpFrame());
      AvatarCommonAsserts.assertDesiredPelvisOrientation(pelvisHomeOrientation, robot, scs, 1.0e-3);
      AvatarCommonAsserts.assertDesiredPelvisAngularVelocityZero(robot, scs, 1.0e-6);
      AvatarCommonAsserts.assertDesiredPelvisHeightOffsetZero(scs, 1.0e-10);
      AvatarCommonAsserts.assertDesiredICPOffsetZero(scs, 1.0e-3);
   }

   private static double[] getArmHome(DRCRobotModel robotModel, RobotSide robotSide, FullHumanoidRobotModel robot)
   {
      RigidBodyBasics chest = robot.getChest();
      RigidBodyBasics hand = robot.getHand(robotSide);
      OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);

      TObjectDoubleHashMap<String> jointHome = robotModel.getWalkingControllerParameters().getOrCreateJointHomeConfiguration();
      Map<String, RigidBodyControlMode> defaultControlModes = robotModel.getWalkingControllerParameters().getDefaultControlModesForRigidBodies();
      if (defaultControlModes.containsKey(hand.getName()))
      {
         Assert.assertEquals("This test assumes the hand is controlled in jointspace by default.", RigidBodyControlMode.JOINTSPACE,
                             defaultControlModes.get(hand.getName()));
      }

      int numberOfJoints = MultiBodySystemTools.computeDegreesOfFreedom(armJoints);
      double[] homePositions = new double[numberOfJoints];
      for (int i = 0; i < armJoints.length; i++)
      {
         homePositions[i] = jointHome.get(armJoints[i].getName());
      }

      return homePositions;
   }

   private static Pose3D getChestHome(DRCRobotModel robotModel, FullHumanoidRobotModel robot)
   {
      RigidBodyBasics chest = robot.getChest();

      Map<String, Pose3D> bodyHome = robotModel.getWalkingControllerParameters().getOrCreateBodyHomeConfiguration();
      Map<String, RigidBodyControlMode> defaultControlModes = robotModel.getWalkingControllerParameters().getDefaultControlModesForRigidBodies();
      Assert.assertEquals("This test assumes the chest is controlled in taskspace by default.", RigidBodyControlMode.TASKSPACE,
                          defaultControlModes.get(chest.getName()));
      Assert.assertTrue("Bodies controlled in taskspace by default must specify a home pose.", bodyHome.containsKey(chest.getName()));

      return bodyHome.get(chest.getName());
   }

   private void createSimulation() throws SimulationExceededMaximumTimeException
   {
      testHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), new FlatGroundEnvironment());
      testHelper.setStartingLocation(new OffsetAndYawRobotInitialSetup(EuclidCoreRandomTools.nextDouble(random, Math.PI)));
      testHelper.createSimulation(getClass().getSimpleName());
      Assert.assertTrue(testHelper.simulateAndBlockAndCatchExceptions(0.25));
   }

   @BeforeEach
   public void setup()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void destroy()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      if (testHelper != null)
      {
         testHelper.destroySimulation();
         testHelper = null;
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }
}
