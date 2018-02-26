package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertTrue;

import java.util.List;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

public abstract class EndToEndHandLoadBearingTest implements MultiRobotTestInterface
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   protected DRCSimulationTestHelper drcSimulationTestHelper;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public void testUsingHand() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      String testName = getClass().getSimpleName();
      TestingEnvironment testingEnvironment = new TestingEnvironment();
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setTestEnvironment(testingEnvironment);
      drcSimulationTestHelper.createSimulation(testName);
      double totalMass = fullRobotModel.getTotalMass();
      PushRobotController pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(), fullRobotModel);
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      scs.setCameraPosition(0.2, -10.0, 1.0);
      scs.setCameraFix(0.2, 0.0, 1.0);
      scs.addYoGraphic(pushRobotController.getForceVisualizer());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      ReferenceFrame chestFrame = referenceFrames.getChestFrame();

      // Position hand above table
      Quaternion chestOrientation = new Quaternion();
      chestOrientation.appendPitchRotation(Math.PI / 4.0);
      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(1.0, chestOrientation, worldFrame, pelvisZUpFrame);
      drcSimulationTestHelper.send(chestTrajectoryMessage);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);
      assertTrue(success);

      Quaternion handOrientation = new Quaternion();
      handOrientation.appendYawRotation(-Math.PI / 2.0);
      handOrientation.appendPitchRotation(Math.PI / 2.0);

      HandTrajectoryMessage handTrajectoryMessage1 = HumanoidMessageTools.createHandTrajectoryMessage(RobotSide.LEFT, 1);
      handTrajectoryMessage1.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrame(chestFrame);
      handTrajectoryMessage1.getSe3Trajectory().getFrameInformation().setDataReferenceFrame(worldFrame);
      handTrajectoryMessage1.getSe3Trajectory().setTrajectoryPoint(0, 1.0, new Point3D(0.45, 0.3, 0.6), handOrientation, new Vector3D(), new Vector3D(), worldFrame);
      drcSimulationTestHelper.send(handTrajectoryMessage1);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);

      HandTrajectoryMessage handTrajectoryMessage2 = HumanoidMessageTools.createHandTrajectoryMessage(RobotSide.LEFT, 1);
      handTrajectoryMessage2.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrame(chestFrame);
      handTrajectoryMessage2.getSe3Trajectory().getFrameInformation().setDataReferenceFrame(worldFrame);
      handTrajectoryMessage2.getSe3Trajectory().setTrajectoryPoint(0, 1.0, new Point3D(0.45, 0.3, 0.55), handOrientation, new Vector3D(), new Vector3D(), worldFrame);
      drcSimulationTestHelper.send(handTrajectoryMessage2);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);
      assertTrue(success);

      // Activate load bearing
      RigidBodyTransform transformToContactFrame = new RigidBodyTransform();
      transformToContactFrame.setTranslation(0.0, 0.09, 0.0);
      transformToContactFrame.appendRollRotation(Math.PI);

      HandLoadBearingMessage loadBearingMessage = HumanoidMessageTools.createHandLoadBearingMessage(RobotSide.LEFT);
      loadBearingMessage.getLoadBearingMessage().setLoad(true);
      loadBearingMessage.getLoadBearingMessage().setCoefficientOfFriction(0.8);
      loadBearingMessage.getLoadBearingMessage().setContactNormalInWorldFrame(new Vector3D(0.0, 0.0, 1.0));
      loadBearingMessage.getLoadBearingMessage().setBodyFrameToContactFrame(transformToContactFrame);
      drcSimulationTestHelper.send(loadBearingMessage);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      // Now push the robot
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double percentWeight = 0.1;
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 2.0;
      pushRobotController.applyForce(forceDirection, magnitude, duration);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      assertTrue(success);
      
      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
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

   @Before
   public void showMemoryUsageBeforeTest()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
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

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

}
