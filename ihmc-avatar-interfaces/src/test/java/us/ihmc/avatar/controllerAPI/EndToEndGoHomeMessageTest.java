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
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
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
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

public abstract class EndToEndGoHomeMessageTest implements MultiRobotTestInterface
{
   private static final Random random = new Random(2943L);

   private SimulationTestingParameters simulationTestingParameters;
   private SCS2AvatarTestingSimulation testHelper;

   public void testGoHomeArms() throws SimulationExceededMaximumTimeException
   {
      createSimulation();

      double trajectoryTime = 0.5;
      FullHumanoidRobotModel robot = testHelper.getControllerFullRobotModel();
      double[] leftHome = getArmHome(getRobotModel(), RobotSide.LEFT, robot);
      double[] rightHome = getArmHome(getRobotModel(), RobotSide.RIGHT, robot);

      testHelper.publishToController(AvatarRandomTestMessages.nextArmTrajectoryMessage(random, trajectoryTime, RobotSide.LEFT, robot));
      testHelper.publishToController(AvatarRandomTestMessages.nextArmTrajectoryMessage(random, trajectoryTime, RobotSide.RIGHT, robot));

      Assert.assertTrue(testHelper.simulateAndWait(trajectoryTime + 0.25));

      testHelper.publishToController(HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.ARM, RobotSide.LEFT, trajectoryTime));
      testHelper.publishToController(HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.ARM, RobotSide.RIGHT, trajectoryTime));

      Assert.assertTrue(testHelper.simulateAndWait(trajectoryTime + 0.25));

      AvatarCommonAsserts.assertDesiredArmPositions(leftHome, RobotSide.LEFT, robot, testHelper, 1.0e-10);
      AvatarCommonAsserts.assertDesiredArmPositions(rightHome, RobotSide.RIGHT, robot, testHelper, 1.0e-10);
      AvatarCommonAsserts.assertDesiredArmVelocitiesZero(RobotSide.LEFT, robot, testHelper, 1.0e-10);
      AvatarCommonAsserts.assertDesiredArmVelocitiesZero(RobotSide.RIGHT, robot, testHelper, 1.0e-10);

      Assert.assertTrue(testHelper.simulateAndWait(2.0));

      AvatarCommonAsserts.assertArmPositions(leftHome, RobotSide.LEFT, robot, Math.toRadians(15.0));
      AvatarCommonAsserts.assertArmPositions(rightHome, RobotSide.RIGHT, robot, Math.toRadians(15.0));
   }

   public void testGoHomeChest() throws SimulationExceededMaximumTimeException
   {
      createSimulation();

      double trajectoryTime = 0.5;
      MovingReferenceFrame pelvisZUpFrame = testHelper.getReferenceFrames().getPelvisZUpFrame();
      FullHumanoidRobotModel robot = testHelper.getControllerFullRobotModel();
      FrameQuaternion chestHome = new FrameQuaternion(pelvisZUpFrame, getChestHome(getRobotModel(), robot).getOrientation());

      testHelper.publishToController(AvatarRandomTestMessages.nextChestTrajectoryMessage(random, trajectoryTime, pelvisZUpFrame, robot));

      Assert.assertTrue(testHelper.simulateAndWait(trajectoryTime + 0.25));

      testHelper.publishToController(HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.CHEST, trajectoryTime));

      Assert.assertTrue(testHelper.simulateAndWait(trajectoryTime + 0.25));

      AvatarCommonAsserts.assertDesiredChestOrientation(chestHome, robot, testHelper, 1.0e-6);
      AvatarCommonAsserts.assertDesiredChestAngularVelocityZero(robot, testHelper, 1.0e-6);

      Assert.assertTrue(testHelper.simulateAndWait(2.0));

      AvatarCommonAsserts.assertChestOrientation(chestHome, robot, Math.toRadians(1.0));
   }

   public void testGoHomePelvis() throws SimulationExceededMaximumTimeException
   {
      createSimulation();

      double trajectoryTime = 1.0;
      FullHumanoidRobotModel robot = testHelper.getControllerFullRobotModel();

      testHelper.publishToController(AvatarRandomTestMessages.nextPelvisTrajectoryMessage(random, trajectoryTime, robot, 0.1, Math.toRadians(30.0)));

      Assert.assertTrue(testHelper.simulateAndWait(trajectoryTime + 0.5));

      testHelper.publishToController(HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.PELVIS, trajectoryTime));

      Assert.assertTrue(testHelper.simulateAndWait(trajectoryTime + 1.0));

      FrameQuaternion pelvisHomeOrientation = new FrameQuaternion(testHelper.getReferenceFrames().getMidFeetZUpFrame());
      AvatarCommonAsserts.assertDesiredPelvisOrientation(pelvisHomeOrientation, robot, testHelper, 1.0e-3);
      AvatarCommonAsserts.assertDesiredPelvisAngularVelocityZero(robot, testHelper, 1.0e-6);
      AvatarCommonAsserts.assertDesiredPelvisHeightOffsetZero(testHelper, 1.0e-10);
      AvatarCommonAsserts.assertDesiredICPOffsetZero(testHelper, 1.0e-3);
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
         Assert.assertEquals("This test assumes the hand is controlled in jointspace by default.",
                             RigidBodyControlMode.JOINTSPACE,
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
      Assert.assertEquals("This test assumes the chest is controlled in taskspace by default.",
                          RigidBodyControlMode.TASKSPACE,
                          defaultControlModes.get(chest.getName()));
      Assert.assertTrue("Bodies controlled in taskspace by default must specify a home pose.", bodyHome.containsKey(chest.getName()));

      return bodyHome.get(chest.getName());
   }

   private void createSimulation() throws SimulationExceededMaximumTimeException
   {
      SCS2AvatarTestingSimulationFactory factory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                         new FlatGroundEnvironment(),
                                                                                                                         simulationTestingParameters);
      factory.setStartingLocationOffset(new OffsetAndYawRobotInitialSetup(EuclidCoreRandomTools.nextDouble(random, Math.PI)));
      testHelper = factory.createAvatarTestingSimulation();
      testHelper.start();
      Assert.assertTrue(testHelper.simulateAndWait(0.25));
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
      if (testHelper != null)
      {
         testHelper.finishTest();
         testHelper = null;
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }
}
