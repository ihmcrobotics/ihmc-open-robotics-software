package us.ihmc.avatar.controllerAPI;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.HandLoadBearingMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.SE3TrajectoryMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

public abstract class EndToEndHandLoadBearingTest implements MultiRobotTestInterface
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   protected SCS2AvatarTestingSimulation simulationTestHelper;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testUsingHand() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      TestingEnvironment testingEnvironment = new TestingEnvironment();
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(robotModel, testingEnvironment, simulationTestingParameters);
      simulationTestHelper.start();
      double totalMass = fullRobotModel.getTotalMass();
      PushRobotControllerSCS2 pushRobotController = new PushRobotControllerSCS2(simulationTestHelper.getSimulationSession().getTime(), simulationTestHelper.getRobot(), fullRobotModel);

      simulationTestHelper.setCameraPosition(0.2, -10.0, 1.0);
      simulationTestHelper.setCameraFocusPosition(0.2, 0.0, 1.0);
      simulationTestHelper.addYoGraphicDefinition(pushRobotController.getForceVizDefinition());

      boolean success = simulationTestHelper.simulateAndWait(0.5);
      assertTrue(success);

      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      ReferenceFrame chestFrame = referenceFrames.getChestFrame();

      // Position hand above table
      Quaternion chestOrientation = new Quaternion();
      chestOrientation.appendPitchRotation(Math.PI / 4.0);
      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(1.0, chestOrientation, worldFrame, pelvisZUpFrame);
      simulationTestHelper.publishToController(chestTrajectoryMessage);
      success = simulationTestHelper.simulateAndWait(1.5);
      assertTrue(success);

      Quaternion handOrientation = new Quaternion();
      handOrientation.appendYawRotation(-Math.PI / 2.0);
      handOrientation.appendPitchRotation(Math.PI / 2.0);

      HandTrajectoryMessage handTrajectoryMessage1 = new HandTrajectoryMessage();
      handTrajectoryMessage1.setRobotSide(RobotSide.LEFT.toByte());
      SE3TrajectoryMessage se3Trajectory1 = handTrajectoryMessage1.getSe3Trajectory();
      se3Trajectory1.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(chestFrame));
      se3Trajectory1.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));
      se3Trajectory1.getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(1.0, new Point3D(0.45, 0.3, 0.6), handOrientation, new Vector3D(), new Vector3D()));
      simulationTestHelper.publishToController(handTrajectoryMessage1);
      success = simulationTestHelper.simulateAndWait(2.0);
      assertTrue(success);

      HandTrajectoryMessage handTrajectoryMessage2 = new HandTrajectoryMessage();
      handTrajectoryMessage2.setRobotSide(RobotSide.LEFT.toByte());
      SE3TrajectoryMessage se3Trajectory2 = handTrajectoryMessage2.getSe3Trajectory();
      se3Trajectory2.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(chestFrame));
      se3Trajectory2.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));
      se3Trajectory2.getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(1.0, new Point3D(0.45, 0.3, 0.55), handOrientation, new Vector3D(), new Vector3D()));
      simulationTestHelper.publishToController(handTrajectoryMessage2);
      success = simulationTestHelper.simulateAndWait(1.5);
      assertTrue(success);

      // Activate load bearing
      Pose3D transformToContactFrame = new Pose3D();
      transformToContactFrame.getPosition().set(0.0, 0.09, 0.0);
      transformToContactFrame.appendRollRotation(Math.PI);

      HandLoadBearingMessage loadBearingMessage = HumanoidMessageTools.createHandLoadBearingMessage(RobotSide.LEFT);
      loadBearingMessage.getLoadBearingMessage().setLoad(true);
      loadBearingMessage.getLoadBearingMessage().setCoefficientOfFriction(0.8);
      loadBearingMessage.getLoadBearingMessage().getContactNormalInWorldFrame().set(0.0, 0.0, 1.0);
      loadBearingMessage.getLoadBearingMessage().getBodyFrameToContactFrame().set(transformToContactFrame);
      simulationTestHelper.publishToController(loadBearingMessage);
      success = simulationTestHelper.simulateAndWait(1.0);
      assertTrue(success);

      // Now push the robot
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double percentWeight = 0.1;
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 2.0;
      pushRobotController.applyForce(forceDirection, magnitude, duration);

      success = simulationTestHelper.simulateAndWait(3.0);
      assertTrue(success);
      
      simulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   public class TestingEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D terrain;

      public TestingEnvironment()
      {
         terrain = new CombinedTerrainObject3D(getClass().getSimpleName());
         YoAppearanceTexture groundTexture = new YoAppearanceTexture("Textures/ground2.png");
         terrain.addBox(-25.0, -25.0, 25.0, 25.0, -0.01, 0.0, groundTexture);
         terrain.addBox(0.3, -0.5, 1.0, 0.5, 0.0, 0.5, YoAppearance.BurlyWood());
      }

      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         return terrain;
      }

      @Override
      public List<? extends Robot> getEnvironmentRobots()
      {
         return null;
      }

      @Override
      public void createAndSetContactControllerToARobot()
      {
      }

      @Override
      public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
      {
      }

      @Override
      public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
      {
      }
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

}
