package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertTrue;

import java.util.List;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndHandLoadBearingTest implements MultiRobotTestInterface
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   protected DRCSimulationTestHelper drcSimulationTestHelper;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public void testUsingHand() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      String testName = getClass().getSimpleName();
      TestingEnvironment testingEnvironment = new TestingEnvironment();
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(testingEnvironment, testName, selectedLocation, simulationTestingParameters, robotModel);
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
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(1.0, chestOrientation, worldFrame, pelvisZUpFrame);
      drcSimulationTestHelper.send(chestTrajectoryMessage);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);
      assertTrue(success);

      Quaternion handOrientation = new Quaternion();
      handOrientation.appendYawRotation(-Math.PI / 2.0);
      handOrientation.appendPitchRotation(Math.PI / 2.0);

      HandTrajectoryMessage handTrajectoryMessage1 = new HandTrajectoryMessage(RobotSide.LEFT, 1);
      handTrajectoryMessage1.setTrajectoryReferenceFrameId(chestFrame);
      handTrajectoryMessage1.setDataReferenceFrameId(worldFrame);
      handTrajectoryMessage1.setTrajectoryPoint(0, 1.0, new Point3D(0.45, 0.3, 0.6), handOrientation, new Vector3D(), new Vector3D(), worldFrame);
      drcSimulationTestHelper.send(handTrajectoryMessage1);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);

      HandTrajectoryMessage handTrajectoryMessage2 = new HandTrajectoryMessage(RobotSide.LEFT, 1);
      handTrajectoryMessage2.setTrajectoryReferenceFrameId(chestFrame);
      handTrajectoryMessage2.setDataReferenceFrameId(worldFrame);
      handTrajectoryMessage2.setTrajectoryPoint(0, 1.0, new Point3D(0.45, 0.3, 0.55), handOrientation, new Vector3D(), new Vector3D(), worldFrame);
      drcSimulationTestHelper.send(handTrajectoryMessage2);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);
      assertTrue(success);

      // Activate load bearing
      RigidBodyTransform transformToContactFrame = new RigidBodyTransform();
      transformToContactFrame.setTranslation(0.0, 0.09, 0.0);
      transformToContactFrame.appendRollRotation(Math.PI);

      HandLoadBearingMessage loadBearingMessage = new HandLoadBearingMessage(RobotSide.LEFT);
      loadBearingMessage.setLoad(true);
      loadBearingMessage.setCoefficientOfFriction(0.8);
      loadBearingMessage.setContactNormalInWorldFrame(new Vector3D(0.0, 0.0, 1.0));
      loadBearingMessage.setBodyFrameToContactFrame(transformToContactFrame);
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
   }

   public void testWalkingWithCane() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      String testName = getClass().getSimpleName();
      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGroundEnvironment, testName, selectedLocation, simulationTestingParameters, getRobotModel());
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      scs.setCameraPosition(-1.0, -3.0, 1.3);
      scs.setCameraFix(0.0, 0.0, 0.3);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(getRobotModel().createFullRobotModel());
      ReferenceFrame chestFrame = referenceFrames.getChestFrame();
      RobotSide robotSide = RobotSide.RIGHT;

      // position the contact point on the ground
      Quaternion handOrientation = new Quaternion();
      handOrientation.appendYawRotation(Math.PI / 4.0);
      handOrientation.appendPitchRotation(-Math.PI / 2.0);
      handOrientation.appendRollRotation(Math.PI / 4.0);
      handOrientation.inverse();

      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, 2);
      handTrajectoryMessage.setTrajectoryReferenceFrameId(chestFrame);
      handTrajectoryMessage.setDataReferenceFrameId(worldFrame);
      handTrajectoryMessage.setUseCustomControlFrame(true);
      handTrajectoryMessage.setControlFramePosition(new Vector3D(-0.307, -0.027, -0.022)); // hard coded to be at the simulation contact point.
      handTrajectoryMessage.setControlFrameOrientation(handOrientation);

      handTrajectoryMessage.setTrajectoryPoint(0, 1.0, new Point3D(0.275, -0.125, 0.1), new Quaternion(), new Vector3D(), new Vector3D(), worldFrame);
      handTrajectoryMessage.setTrajectoryPoint(1, 2.0, new Point3D(0.275, -0.125, -0.1), new Quaternion(), new Vector3D(), new Vector3D(), worldFrame);
      drcSimulationTestHelper.send(handTrajectoryMessage);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // Activate load bearing
      RigidBodyTransform transformToContactFrame = new RigidBodyTransform();
      transformToContactFrame.setTranslation(-0.307, -0.027, -0.022); // hard coded to be at the simulation contact point.

      HandLoadBearingMessage loadBearingMessage = new HandLoadBearingMessage(robotSide);
      loadBearingMessage.setLoad(true);
      loadBearingMessage.setCoefficientOfFriction(0.8);
      loadBearingMessage.setContactNormalInWorldFrame(new Vector3D(0.0, 0.0, 1.0));
      loadBearingMessage.setBodyFrameToContactFrame(transformToContactFrame);
      drcSimulationTestHelper.send(loadBearingMessage);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      // Lift a foot and move the pelvis forward
      String namespace = PelvisICPBasedTranslationManager.class.getSimpleName();
      DoubleYoVariable supportPolygonSafeMargin = (DoubleYoVariable) scs.getVariable(namespace, "supportPolygonSafeMargin");
      supportPolygonSafeMargin.set(-0.1);

      FootTrajectoryMessage footTrajectory = new FootTrajectoryMessage(robotSide, 1);
      Quaternion footOrientation = new Quaternion();
      Point3D footPosition = new Point3D(0.25, 0.05, 0.2);
      footOrientation.appendPitchRotation(-Math.PI / 4.0);
      footTrajectory.setTrajectoryPoint(0, 1.0, footPosition, footOrientation, new Vector3D(), new Vector3D());
      drcSimulationTestHelper.send(footTrajectory);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.25);
      assertTrue(success);

      PelvisTrajectoryMessage pelvisTrajectory = new PelvisTrajectoryMessage(1);
      Point3D pelvisPosition = new Point3D(0.15, 0.0, 0.4);
      pelvisTrajectory.setTrajectoryPoint(0, 0.75, pelvisPosition, new Quaternion(), new Vector3D(), new Vector3D());
      drcSimulationTestHelper.send(pelvisTrajectory);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      assertTrue(success);

      // Return to safe stance and lift hand
      footOrientation.setToZero();
      FootTrajectoryMessage putDownFoot = new FootTrajectoryMessage(robotSide, 2);
      putDownFoot.setTrajectoryPoint(0, 1.0, new Point3D(0.0, -0.075, 0.05), footOrientation, new Vector3D(), new Vector3D());
      putDownFoot.setTrajectoryPoint(1, 1.5, new Point3D(0.0, -0.075, -0.01), footOrientation, new Vector3D(), new Vector3D());
      drcSimulationTestHelper.send(putDownFoot);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.25);
      assertTrue(success);

      pelvisTrajectory.setTrajectoryPoint(0, 0.75, new Point3D(0.0, 0.0, 0.3), new Quaternion(), new Vector3D(), new Vector3D());
      drcSimulationTestHelper.send(pelvisTrajectory);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      HandTrajectoryMessage liftHand = new HandTrajectoryMessage(robotSide, 1);
      liftHand.setTrajectoryReferenceFrameId(chestFrame);
      liftHand.setDataReferenceFrameId(worldFrame);
      liftHand.setUseCustomControlFrame(true);
      liftHand.setControlFrameOrientation(handOrientation);
      liftHand.setTrajectoryPoint(0, 0.5, new Point3D(0.2, -0.15, 0.5), new Quaternion(), new Vector3D(), new Vector3D(), worldFrame);
      drcSimulationTestHelper.send(liftHand);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
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
