package us.ihmc.avatar.roughTerrainWalking;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.trajectories.PositionOptimizedTrajectoryGenerator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class AvatarSwingWithWaypointsTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SCS2AvatarTestingSimulation simulationTestHelper;

   @Test
   public void testCrazySwingIsRejected()
   {
      DRCRobotModel robotModel = getRobotModel();

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(robotModel,
                                                                                            new FlatGroundEnvironment(),
                                                                                            simulationTestingParameters);
      simulationTestHelper.start();
      simulationTestHelper.setCamera(new Point3D(0.0, -0.2, 0.3), new Point3D(0.0, 3.8, 0.15));
      Assert.assertTrue(simulationTestHelper.simulateNow(0.25));

      simulationTestHelper.findVariable("MaxStepDistance").setValueFromDouble(1.0);
      simulationTestHelper.findVariable("MaxSwingDistance").setValueFromDouble(0.5);

      RobotSide robotSide = RobotSide.LEFT;
      MovingReferenceFrame soleFrame = simulationTestHelper.getControllerReferenceFrames().getSoleFrame(robotSide);
      FramePose3D initialPose = new FramePose3D(soleFrame);
      initialPose.changeFrame(ReferenceFrame.getWorldFrame());

      // Test long step is rejected.
      {
         FramePose3D footstepPose = new FramePose3D(initialPose);
         footstepPose.getPosition().addX(2.0);
         FootstepDataListMessage footsteps = new FootstepDataListMessage();
         FootstepDataMessage footstepData = footsteps.getFootstepDataList().add();
         footstepData.setRobotSide(robotSide.toByte());
         footstepData.getLocation().set(footstepPose.getPosition());
         footstepData.getOrientation().set(footstepPose.getOrientation());

         simulationTestHelper.publishToController(footsteps);
         Assert.assertTrue(simulationTestHelper.simulateNow(0.25));
         Assert.assertEquals(0, simulationTestHelper.findVariable("currentNumberOfFootsteps").getValueAsLongBits());
      }

      // Test weird swing is rejected.
      {
         FramePose3D footstepPose = new FramePose3D(initialPose);
         FootstepDataListMessage footsteps = new FootstepDataListMessage();
         FootstepDataMessage footstepData = footsteps.getFootstepDataList().add();
         footstepData.setRobotSide(robotSide.toByte());
         footstepData.getLocation().set(footstepPose.getPosition());
         footstepData.getOrientation().set(footstepPose.getOrientation());

         footstepData.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
         footstepData.getCustomPositionWaypoints().add().set(footstepPose.getPosition());
         footstepData.getCustomPositionWaypoints().add().set(footstepPose.getPosition());

         footstepData.getCustomPositionWaypoints().get(0).addY(1.0);

         simulationTestHelper.publishToController(footsteps);
         Assert.assertTrue(simulationTestHelper.simulateNow(0.25));
         Assert.assertEquals(0, simulationTestHelper.findVariable("currentNumberOfFootsteps").getValueAsLongBits());
      }
   }

   @Test
   public void testSwingWithWaypointsAndNotTrustingHeight()
   {
      DRCRobotModel robotModel = getRobotModel();

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(robotModel,
                                                                                            new FlatGroundEnvironment(),
                                                                                            simulationTestingParameters);
      simulationTestHelper.start();
      simulationTestHelper.setCamera(new Point3D(0.0, -0.2, 0.3), new Point3D(0.0, 3.8, 0.15));
      Assert.assertTrue(simulationTestHelper.simulateNow(0.25));

      simulationTestHelper.findVariable("blindFootstepsHeightOffset").setValueFromDouble(0.0);

      RobotSide robotSide = RobotSide.LEFT;
      MovingReferenceFrame soleFrame = simulationTestHelper.getControllerReferenceFrames().getSoleFrame(robotSide);
      FramePose3D initialPose = new FramePose3D(soleFrame);
      initialPose.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose3D footstepPose = new FramePose3D(initialPose);
      footstepPose.getPosition().addX(0.2);
      footstepPose.getPosition().addZ(0.2);

      FootstepDataListMessage footsteps = new FootstepDataListMessage();
      FootstepDataMessage footstepData = footsteps.getFootstepDataList().add();
      footstepData.setRobotSide(robotSide.toByte());
      footstepData.getLocation().set(footstepPose.getPosition());
      footstepData.getOrientation().set(footstepPose.getOrientation());

      int waypoints = 4;
      double swingDuration = 1.0;
      footstepData.setSwingDuration(swingDuration);
      footstepData.setTrajectoryType(TrajectoryType.WAYPOINTS.toByte());

      List<FramePoint3DReadOnly> waypointPositions = new ArrayList<>();
      for (int i = 0; i < waypoints; i++)
      {
         double percent = (double) (i + 1) / (double) (waypoints + 1);
         FramePose3D waypoint = new FramePose3D();
         waypoint.interpolate(initialPose, footstepPose, percent);
         waypoint.getPosition().addZ(0.1 * Math.sin(percent * Math.PI));
         waypointPositions.add(waypoint.getPosition());
      }

      PositionOptimizedTrajectoryGenerator generator = new PositionOptimizedTrajectoryGenerator("", new YoRegistry("Dummy"), null, 100, waypoints);
      generator.setEndpointConditions(initialPose.getPosition(), new FrameVector3D(), footstepPose.getPosition(), new FrameVector3D());
      generator.setWaypoints(waypointPositions);
      generator.initialize();

      for (int i = 0; i < waypoints; i++)
      {
         SE3TrajectoryPointMessage swingWaypoint = footstepData.getSwingTrajectory().add();
         swingWaypoint.getPosition().set(waypointPositions.get(i));
         swingWaypoint.getOrientation().set(initialPose.getOrientation());
         swingWaypoint.setTime(generator.getWaypointTime(i) * swingDuration);

         FrameVector3D velocity = new FrameVector3D();
         generator.getWaypointVelocity(i, velocity);
         velocity.scale(1.0 / swingDuration);
         swingWaypoint.getLinearVelocity().set(velocity);
      }

      SE3TrajectoryPointMessage swingWaypoint = footstepData.getSwingTrajectory().add();
      swingWaypoint.getPosition().set(footstepPose.getPosition());
      swingWaypoint.getOrientation().set(footstepPose.getOrientation());
      swingWaypoint.setTime(swingDuration);

      footsteps.setTrustHeightOfFootsteps(false);

      simulationTestHelper.publishToController(footsteps);
      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      Assert.assertTrue(simulationTestHelper.simulateNow(initialTransfer + swingDuration / 2.0));

      double waypointHeight = simulationTestHelper.findVariable("SwingWaypoint" + robotSide.getPascalCaseName() + waypoints + "Z").getValueAsDouble();
      Assert.assertEquals(0.0, waypointHeight, 0.05);

      Assert.assertTrue(simulationTestHelper.simulateNow(swingDuration));
   }

   @Test
   public void testRegularSwingWithWaypoints()
   {
      TestingEnvironment environment = new TestingEnvironment();
      DRCRobotModel robotModel = getRobotModel();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(robotModel, environment, simulationTestingParameters);
      simulationTestHelper.start();
      simulationTestHelper.setCameraPosition(8.0, -8.0, 5.0);
      simulationTestHelper.setCameraFocusPosition(1.5, 0.0, 0.8);
      ThreadTools.sleep(1000);
      simulationTestHelper.simulateNow(0.5);

      double swingTime = 2.0;
      double transferTime = 0.8;
      double stepLength = 0.3;
      double stepWidth = 0.14;
      int steps = 10;

      FootstepDataListMessage footsteps = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      for (int i = 1; i <= steps; i++)
      {
         RobotSide robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double footstepX = stepLength * i;
         Point3D location = new Point3D(footstepX, footstepY, 0.0);
         Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);
         footsteps.getFootstepDataList().add().set(footstepData);
      }

      // regular footstep
      {
         int i = 1;
         RobotSide robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double footstepX = stepLength * i;
         double swingHeight = 0.1;

         FootstepDataMessage footstep1 = footsteps.getFootstepDataList().get(i - 1);
         footstep1.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
         MessageTools.copyData(new Point3D[] {new Point3D(footstepX - (stepLength * 0.85), footstepY, swingHeight),
               new Point3D(footstepX - (stepLength * 0.15), footstepY, swingHeight)}, footstep1.getCustomPositionWaypoints());
      }

      // straight up and down
      {
         int i = 2;
         RobotSide robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double footstepX = stepLength * i;

         FootstepDataMessage footstep1 = footsteps.getFootstepDataList().get(i - 1);
         footstep1.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
         MessageTools.copyData(new Point3D[] {new Point3D(footstepX - (stepLength * 2.0), footstepY, 0.25),
               new Point3D(footstepX - (stepLength * 0.0), footstepY, 0.2)}, footstep1.getCustomPositionWaypoints());
      }

      // overshoot
      {
         int i = 3;
         RobotSide robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double footstepX = stepLength * i;

         FootstepDataMessage footstep1 = footsteps.getFootstepDataList().get(i - 1);
         footstep1.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
         MessageTools.copyData(new Point3D[] {new Point3D(footstepX - (stepLength * 2.0 * 0.85), footstepY, 0.2),
               new Point3D(footstepX + 0.1, footstepY, 0.125)}, footstep1.getCustomPositionWaypoints());
      }

      // swing outward sideways
      {
         int i = 6;
         RobotSide robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double offsetY = robotSide == RobotSide.LEFT ? 0.2 : -0.2;
         double footstepX = stepLength * i;
         double swingHeight = 0.15;

         FootstepDataMessage footstep1 = footsteps.getFootstepDataList().get(i - 1);
         footstep1.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
         MessageTools.copyData(new Point3D[] {new Point3D(footstepX - (stepLength * 2.0 * 0.85), footstepY + offsetY, swingHeight),
               new Point3D(footstepX - (stepLength * 2.0 * 0.15), footstepY + offsetY, swingHeight)}, footstep1.getCustomPositionWaypoints());
      }

      // crazy
      {
         int i = 7;
         RobotSide robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double footstepX = stepLength * i;
         double swingHeight = 0.15;

         FootstepDataMessage footstep1 = footsteps.getFootstepDataList().get(i - 1);
         footstep1.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
         MessageTools.copyData(new Point3D[] {new Point3D(footstepX - (stepLength * 2.0 * 0.7), footstepY - 0.15, swingHeight + 0.04),
               new Point3D(footstepX - (stepLength * 2.0 * 0.2), footstepY, swingHeight + 0.02)}, footstep1.getCustomPositionWaypoints());
      }

      // side to side
      {
         int i = 8;
         RobotSide robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double offsetY = robotSide == RobotSide.LEFT ? 0.15 : -0.15;
         double footstepX = stepLength * i;
         double swingHeight = 0.15;

         FootstepDataMessage footstep1 = footsteps.getFootstepDataList().get(i - 1);
         footstep1.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
         MessageTools.copyData(new Point3D[] {new Point3D(footstepX - (stepLength * 2.0 * 0.85), footstepY + offsetY, swingHeight),
               new Point3D(footstepX - (stepLength * 2.0 * 0.15), footstepY - offsetY, swingHeight)}, footstep1.getCustomPositionWaypoints());
      }

      simulationTestHelper.publishToController(footsteps);
      double simulationTime = (swingTime + transferTime) * steps + 1.0;
      simulationTestHelper.simulateNow(simulationTime);

      Point3D rootJointPosition = new Point3D(2.81, 0.0, 0.82);
      Vector3D epsilon = new Vector3D(0.05, 0.05, 0.10);
      Point3D min = new Point3D(rootJointPosition);
      Point3D max = new Point3D(rootJointPosition);
      min.sub(epsilon);
      max.add(epsilon);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(new BoundingBox3D(min, max));

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);
   }

   @Test
   public void testSwingWithWaypointsRotated()
   {
      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      DRCStartingLocation startingLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;
      DRCRobotModel robotModel = getRobotModel();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             environment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(startingLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      simulationTestHelper.simulateNow(0.5);

      double swingTime = 2.0;
      double transferTime = 0.8;
      double stepLength = 0.3;
      double stepWidth = 0.15;
      double swingHeight = 0.1;
      RobotSide robotSide = RobotSide.LEFT;

      double yaw = startingLocation.getStartingLocationOffset().getYaw();
      RigidBodyTransformGenerator generator = new RigidBodyTransformGenerator();
      generator.rotate(yaw, Axis3D.Z);
      RigidBodyTransform transform = generator.getRigidBodyTransformCopy();

      FootstepDataListMessage footsteps = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
      Point3D stepPosition = new Point3D(stepLength, footstepY, 0.0);
      Quaternion stepOrientation = new Quaternion();
      transform.transform(stepPosition);
      stepOrientation.set(transform.getRotation());
      FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, stepPosition, stepOrientation);
      footsteps.getFootstepDataList().add().set(footstepData);

      // this should be a regular step
      Point3D waypoint1 = new Point3D(stepLength * 0.15, footstepY, swingHeight);
      Point3D waypoint2 = new Point3D(stepLength * 0.85, footstepY, swingHeight);
      transform.transform(waypoint1);
      transform.transform(waypoint2);

      footstepData.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
      MessageTools.copyData(new Point3D[] {waypoint1, waypoint2}, footstepData.getCustomPositionWaypoints());

      simulationTestHelper.publishToController(footsteps);
      double simulationTime = swingTime + transferTime + 1.0;
      simulationTestHelper.simulateNow(simulationTime);
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
      simulationTestingParameters = null;
   }

   public class TestingEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D terrain;

      public TestingEnvironment()
      {
         terrain = new CombinedTerrainObject3D(getClass().getSimpleName());
         terrain.addBox(-0.2, -0.225, 3.2, 0.225, -0.1, 0.0);
         terrain.addBox(0.15, 0.05, 0.45, 0.25, 0.15);
         terrain.addBox(0.6, -0.05, 0.775, -0.25, 0.08);
         terrain.addCylinder(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), new Point3D(1.5, 0.15, 0.1)), 0.2, 0.15, YoAppearance.Grey());
         terrain.addCylinder(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), new Point3D(1.8, -0.15, 0.1)), 0.2, 0.025, YoAppearance.Grey());
         terrain.addBox(1.96, 0.125, 1.99, 0.0, 0.2);
         terrain.addBox(2.235, 0.175, 2.265, 0.25, 0.2);
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
}
