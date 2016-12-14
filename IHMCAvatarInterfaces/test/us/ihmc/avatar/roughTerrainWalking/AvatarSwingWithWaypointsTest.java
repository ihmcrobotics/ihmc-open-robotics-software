package us.ihmc.avatar.roughTerrainWalking;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.thread.ThreadTools;

public abstract class AvatarSwingWithWaypointsTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private DRCSimulationTestHelper drcSimulationTestHelper;

   public void testRegularSwingWithWaypoints() throws SimulationExceededMaximumTimeException
   {
      String className = getClass().getSimpleName();
      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      DRCStartingLocation startingLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, className, startingLocation, simulationTestingParameters, robotModel);
      ThreadTools.sleep(1000);
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
         Point3d location = new Point3d(footstepX, footstepY, 0.0);
         Quat4d orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
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
         footstep1.setTrajectoryWaypoints(new Point3d[] {
               new Point3d(footstepX - (stepLength * 0.85), footstepY, swingHeight),
               new Point3d(footstepX - (stepLength * 0.15), footstepY, swingHeight)});
      }

      // straight up and down
      {
         int i = 2;
         RobotSide robotSide = i%2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double footstepX = stepLength * i;
         double swingHeight = 0.1;

         FootstepDataMessage footstep1 = footsteps.get(i-1);
         footstep1.setTrajectoryType(TrajectoryType.CUSTOM);
         footstep1.setTrajectoryWaypoints(new Point3d[] {
               new Point3d(footstepX - (stepLength * 2.0), footstepY, swingHeight),
               new Point3d(footstepX - (stepLength * 0.0), footstepY, swingHeight)});
      }

      // overshoot
      {
         int i = 4;
         RobotSide robotSide = i%2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double footstepX = stepLength * i;
         double swingHeight = 0.1;

         FootstepDataMessage footstep1 = footsteps.get(i-1);
         footstep1.setTrajectoryType(TrajectoryType.CUSTOM);
         footstep1.setTrajectoryWaypoints(new Point3d[] {
               new Point3d(footstepX - (stepLength * 2.0 * 0.85), footstepY, swingHeight),
               new Point3d(footstepX + 0.1, footstepY, swingHeight)});
      }

      // swing outward sideways
      {
         int i = 6;
         RobotSide robotSide = i%2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double offsetY = robotSide == RobotSide.LEFT ? 0.2 : -0.2;
         double footstepX = stepLength * i;
         double swingHeight = 0.1;

         FootstepDataMessage footstep1 = footsteps.get(i-1);
         footstep1.setTrajectoryType(TrajectoryType.CUSTOM);
         footstep1.setTrajectoryWaypoints(new Point3d[] {
               new Point3d(footstepX - (stepLength * 2.0 * 0.85), footstepY + offsetY, swingHeight),
               new Point3d(footstepX - (stepLength * 2.0 * 0.15), footstepY + offsetY, swingHeight)});
      }

      // crazy
      {
         int i = 7;
         RobotSide robotSide = i%2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double footstepX = stepLength * i;
         double swingHeight = 0.1;

         FootstepDataMessage footstep1 = footsteps.get(i-1);
         footstep1.setTrajectoryType(TrajectoryType.CUSTOM);
         footstep1.setTrajectoryWaypoints(new Point3d[] {
               new Point3d(footstepX - (stepLength * 2.0 * 0.7), footstepY - 0.15, swingHeight + 0.04),
               new Point3d(footstepX - (stepLength * 2.0 * 0.2), footstepY, swingHeight + 0.02)});
      }

      // side to side
      {
         int i = 8;
         RobotSide robotSide = i%2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double offsetY = robotSide == RobotSide.LEFT ? 0.15 : -0.15;
         double footstepX = stepLength * i;
         double swingHeight = 0.1;

         FootstepDataMessage footstep1 = footsteps.get(i-1);
         footstep1.setTrajectoryType(TrajectoryType.CUSTOM);
         footstep1.setTrajectoryWaypoints(new Point3d[] {
               new Point3d(footstepX - (stepLength * 2.0 * 0.85), footstepY + offsetY, swingHeight),
               new Point3d(footstepX - (stepLength * 2.0 * 0.15), footstepY - offsetY, swingHeight)});
      }

      drcSimulationTestHelper.send(footsteps);
      double simulationTime = (swingTime + transferTime) * steps + 1.0;
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
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
      Point3d stepPosition = new Point3d(stepLength, footstepY, 0.0);
      Quat4d stepOrientation = new Quat4d();
      transform.transform(stepPosition);
      transform.getRotation(stepOrientation);
      FootstepDataMessage footstepData = new FootstepDataMessage(robotSide, stepPosition, stepOrientation);
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footsteps.add(footstepData);

      // this should be a regular step
      Point3d waypoint1 = new Point3d(stepLength * 0.15, footstepY, swingHeight);
      Point3d waypoint2 = new Point3d(stepLength * 0.85, footstepY, swingHeight);
      transform.transform(waypoint1);
      transform.transform(waypoint2);

      footstepData.setTrajectoryType(TrajectoryType.CUSTOM);
      footstepData.setTrajectoryWaypoints(new Point3d[] {waypoint1, waypoint2});

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
}
