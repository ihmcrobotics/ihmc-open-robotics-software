package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertTrue;

import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndHandLoadBearingTest implements MultiRobotTestInterface
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   protected DRCSimulationTestHelper drcSimulationTestHelper;

   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 50000)
   public void testUsingHand() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      String testName = getClass().getSimpleName();
      TestingEnvironment testingEnvironment = new TestingEnvironment();
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(testingEnvironment, testName, selectedLocation, simulationTestingParameters, robotModel);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraPosition(0.2, -10.0, 1.0);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraFix(0.2, 0.0, 1.0);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      // Position hand above table
      Quaternion chestOrientation = new Quaternion();
      chestOrientation.appendPitchRotation(Math.PI / 4.0);
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(1.0, chestOrientation);
      drcSimulationTestHelper.send(chestTrajectoryMessage);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);

      Quaternion handOrientation = new Quaternion();
      handOrientation.appendYawRotation(-Math.PI / 2.0);
      handOrientation.appendPitchRotation(Math.PI / 2.0);

      HandTrajectoryMessage handTrajectoryMessage1 = new HandTrajectoryMessage(RobotSide.LEFT, 1);
      handTrajectoryMessage1.setTrajectoryPoint(0, 1.0, new Point3D(0.6, 0.3, 0.625), handOrientation, new Vector3D(), new Vector3D());
      drcSimulationTestHelper.send(handTrajectoryMessage1);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);

      HandTrajectoryMessage handTrajectoryMessage2 = new HandTrajectoryMessage(RobotSide.LEFT, 1);
      handTrajectoryMessage2.setTrajectoryPoint(0, 1.0, new Point3D(0.6, 0.3, 0.525), handOrientation, new Vector3D(), new Vector3D());
      drcSimulationTestHelper.send(handTrajectoryMessage2);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);

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
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);
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
