package us.ihmc.avatar.roughTerrainWalking;

import java.util.List;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionsettools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.thread.ThreadTools;

public abstract class AvatarSwingWithWaypointsTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private DRCSimulationTestHelper drcSimulationTestHelper;

   public void testRegularSwingWithWaypoints() throws SimulationExceededMaximumTimeException
   {
      String className = getClass().getSimpleName();
      TestingEnvironment environment = new TestingEnvironment();
      DRCStartingLocation startingLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, className, startingLocation, simulationTestingParameters, robotModel);
      ThreadTools.sleep(1000);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraPosition(8.0, -8.0, 5.0);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraFix(1.5, 0.0, 0.8);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      double swingTime = 2.0;
      double transferTime = 0.8;
      double stepLength = 0.3;
      double stepWidth = 0.14;
      int steps = 10;

      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime, transferTime);
      for (int i = 1; i <= steps; i++)
      {
         RobotSide robotSide = i%2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double footstepX = stepLength * i;
         Point3D location = new Point3D(footstepX, footstepY, 0.0);
         Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         FootstepDataMessage footstepData = new FootstepDataMessage(robotSide, location, orientation);
         footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         footsteps.add(footstepData);
      }

      // regular footstep
      {
         int i = 1;
         RobotSide robotSide = i%2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double footstepX = stepLength * i;
         double swingHeight = 0.1;

         FootstepDataMessage footstep1 = footsteps.get(i-1);
         footstep1.setTrajectoryType(TrajectoryType.CUSTOM);
         footstep1.setTrajectoryWaypoints(new Point3D[] {
               new Point3D(footstepX - (stepLength * 0.85), footstepY, swingHeight),
               new Point3D(footstepX - (stepLength * 0.15), footstepY, swingHeight)});
      }

      // straight up and down
      {
         int i = 2;
         RobotSide robotSide = i%2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double footstepX = stepLength * i;

         FootstepDataMessage footstep1 = footsteps.get(i-1);
         footstep1.setTrajectoryType(TrajectoryType.CUSTOM);
         footstep1.setTrajectoryWaypoints(new Point3D[] {
               new Point3D(footstepX - (stepLength * 2.0), footstepY, 0.25),
               new Point3D(footstepX - (stepLength * 0.0), footstepY, 0.2)});
      }

      // overshoot
      {
         int i = 3;
         RobotSide robotSide = i%2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double footstepX = stepLength * i;

         FootstepDataMessage footstep1 = footsteps.get(i-1);
         footstep1.setTrajectoryType(TrajectoryType.CUSTOM);
         footstep1.setTrajectoryWaypoints(new Point3D[] {
               new Point3D(footstepX - (stepLength * 2.0 * 0.85), footstepY, 0.2),
               new Point3D(footstepX + 0.1, footstepY, 0.125)});
      }

      // swing outward sideways
      {
         int i = 6;
         RobotSide robotSide = i%2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double offsetY = robotSide == RobotSide.LEFT ? 0.2 : -0.2;
         double footstepX = stepLength * i;
         double swingHeight = 0.15;

         FootstepDataMessage footstep1 = footsteps.get(i-1);
         footstep1.setTrajectoryType(TrajectoryType.CUSTOM);
         footstep1.setTrajectoryWaypoints(new Point3D[] {
               new Point3D(footstepX - (stepLength * 2.0 * 0.85), footstepY + offsetY, swingHeight),
               new Point3D(footstepX - (stepLength * 2.0 * 0.15), footstepY + offsetY, swingHeight)});
      }

      // crazy
      {
         int i = 7;
         RobotSide robotSide = i%2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double footstepX = stepLength * i;
         double swingHeight = 0.15;

         FootstepDataMessage footstep1 = footsteps.get(i-1);
         footstep1.setTrajectoryType(TrajectoryType.CUSTOM);
         footstep1.setTrajectoryWaypoints(new Point3D[] {
               new Point3D(footstepX - (stepLength * 2.0 * 0.7), footstepY - 0.15, swingHeight + 0.04),
               new Point3D(footstepX - (stepLength * 2.0 * 0.2), footstepY, swingHeight + 0.02)});
      }

      // side to side
      {
         int i = 8;
         RobotSide robotSide = i%2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double offsetY = robotSide == RobotSide.LEFT ? 0.15 : -0.15;
         double footstepX = stepLength * i;
         double swingHeight = 0.15;

         FootstepDataMessage footstep1 = footsteps.get(i-1);
         footstep1.setTrajectoryType(TrajectoryType.CUSTOM);
         footstep1.setTrajectoryWaypoints(new Point3D[] {
               new Point3D(footstepX - (stepLength * 2.0 * 0.85), footstepY + offsetY, swingHeight),
               new Point3D(footstepX - (stepLength * 2.0 * 0.15), footstepY - offsetY, swingHeight)});
      }

      drcSimulationTestHelper.send(footsteps);
      double simulationTime = (swingTime + transferTime) * steps + 1.0;
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);

      Point3D rootJointPosition = new Point3D(2.81, 0.0, 0.79);
      Vector3D epsilon = new Vector3D(0.05, 0.05, 0.05);
      Point3D min = new Point3D(rootJointPosition);
      Point3D max = new Point3D(rootJointPosition);
      min.sub(epsilon);
      max.add(epsilon);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(new BoundingBox3D(min, max));

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   public void testSwingWithWaypointsRotated() throws SimulationExceededMaximumTimeException
   {
      String className = getClass().getSimpleName();
      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      DRCStartingLocation startingLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, className, startingLocation, simulationTestingParameters, robotModel);
      ThreadTools.sleep(1000);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      double swingTime = 2.0;
      double transferTime = 0.8;
      double stepLength = 0.3;
      double stepWidth = 0.15;
      double swingHeight = 0.1;
      RobotSide robotSide = RobotSide.LEFT;

      double yaw = startingLocation.getStartingLocationOffset().getYaw();
      RigidBodyTransformGenerator generator = new RigidBodyTransformGenerator();
      generator.rotate(yaw, Axis.Z);
      RigidBodyTransform transform = generator.getRigidBodyTransformCopy();

      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime, transferTime);
      double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
      Point3D stepPosition = new Point3D(stepLength, footstepY, 0.0);
      Quaternion stepOrientation = new Quaternion();
      transform.transform(stepPosition);
      transform.getRotation(stepOrientation);
      FootstepDataMessage footstepData = new FootstepDataMessage(robotSide, stepPosition, stepOrientation);
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footsteps.add(footstepData);

      // this should be a regular step
      Point3D waypoint1 = new Point3D(stepLength * 0.15, footstepY, swingHeight);
      Point3D waypoint2 = new Point3D(stepLength * 0.85, footstepY, swingHeight);
      transform.transform(waypoint1);
      transform.transform(waypoint2);

      footstepData.setTrajectoryType(TrajectoryType.CUSTOM);
      footstepData.setTrajectoryWaypoints(new Point3D[] {waypoint1, waypoint2});

      drcSimulationTestHelper.send(footsteps);
      double simulationTime = swingTime + transferTime + 1.0;
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
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
