package us.ihmc.avatar.roughTerrainWalking;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public abstract class AvatarSwingOverPlanarRegionsTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private DRCSimulationTestHelper drcSimulationTestHelper;

   private static final boolean LOCAL_MODE = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

   public void testSwingOverPlanarRegions() throws SimulationExceededMaximumTimeException
   {
      String className = getClass().getSimpleName();

      double swingTime = 0.6;
      double transferTime = 0.25;
      double stepLength = 0.3;
      double stepWidth = 0.14;
      double maxSwingSpeed = 1.0;
      int steps = 10;

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      generator.translate(2.0, 0.0, -0.01);
      generator.addCubeReferencedAtCenter(6.0, 1.0, 0.00005);
      generator.translate(-2.0, 0.0, 0.0);
      generator.translate(0.35, 0.2, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.1, 0.1, 0.1);
      generator.translate(0.62, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.1, 0.1, 0.14);
      generator.translate(0.3, -0.3, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.1, 0.1, 0.15);
      generator.translate(0.4, 0.1, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.1, 1.0, 0.11);
      //      generator.translate(0.0, 0.28, 0.0);
      //      generator.addCubeReferencedAtBottomMiddle(0.1, 0.1, 0.14);

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment(planarRegionsList, 1e-2, false);

      DRCStartingLocation startingLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, className, startingLocation, simulationTestingParameters, robotModel);
      ThreadTools.sleep(1000);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraPosition(8.0, -8.0, 5.0);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraFix(1.5, 0.0, 0.8);

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      RobotContactPointParameters contactPointParameters = robotModel.getContactPointParameters();
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();

      AvatarSwingOverPlanarRegionsVisualizer swingOverPlanarRegionsVisualizer = null;
      SwingOverPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;
      if (LOCAL_MODE)
      {
         swingOverPlanarRegionsVisualizer = new AvatarSwingOverPlanarRegionsVisualizer(drcSimulationTestHelper.getSimulationConstructionSet(), registry,
                                                                                       yoGraphicsListRegistry, walkingControllerParameters,
                                                                                       contactPointParameters);
         swingOverPlanarRegionsTrajectoryExpander = swingOverPlanarRegionsVisualizer.getSwingOverPlanarRegionsTrajectoryExpander();
      }
      else
      {
         swingOverPlanarRegionsTrajectoryExpander = new SwingOverPlanarRegionsTrajectoryExpander(walkingControllerParameters, registry, yoGraphicsListRegistry);
      }

      drcSimulationTestHelper.addChildRegistry(registry);
      drcSimulationTestHelper.getSimulationConstructionSet().addYoGraphicsListRegistry(yoGraphicsListRegistry);
      SideDependentList<ConvexPolygon2d> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         footPolygons.set(side, new ConvexPolygon2d(contactPointParameters.getFootContactPoints().get(side)));
      }

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      FramePose stanceFootPose = new FramePose();
      FramePose swingStartPose = new FramePose();
      FramePose swingEndPose = new FramePose();

      stanceFootPose.setPosition(0.0, -stepWidth, 0.0);
      swingEndPose.setPosition(0.0, stepWidth, 0.0);

      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime, transferTime);
      double simulationTime = transferTime * steps + 1.0;
      for (int i = 1; i <= steps; i++)
      {
         RobotSide robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide.negateIfRightSide(stepWidth);
         double footstepX = stepLength * i;
         Point3d location = new Point3d(footstepX, footstepY, 0.0);
         Quat4d orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
         FootstepDataMessage footstepData = new FootstepDataMessage(robotSide, location, orientation);
         footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);

         swingStartPose.set(stanceFootPose);
         stanceFootPose.set(swingEndPose);
         swingEndPose.setPosition(footstepX, footstepY, 0.0);
         double maxSpeedDimensionless = Double.NaN;

         if (LOCAL_MODE)
         {
            maxSpeedDimensionless = swingOverPlanarRegionsVisualizer.expandTrajectoryOverPlanarRegions(stanceFootPose, swingStartPose, swingEndPose, planarRegionsList);
         }
         else
         {
            maxSpeedDimensionless = swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(stanceFootPose, swingStartPose, swingEndPose, planarRegionsList);
         }

         PrintTools.info("Step " + i + ": " + swingOverPlanarRegionsTrajectoryExpander.getStatus());
         PrintTools.info("Foot: " + robotSide + "  X: " + footstepX + "  Y: " + footstepY);

         footstepData.setTrajectoryType(TrajectoryType.CUSTOM);
         Point3d waypointOne = new Point3d();
         Point3d waypointTwo = new Point3d();
         swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(0).get(waypointOne);
         swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(1).get(waypointTwo);
         footstepData.setTrajectoryWaypoints(new Point3d[] {waypointOne, waypointTwo});

         double maxSpeed = maxSpeedDimensionless / swingTime;
         if (maxSpeed > maxSwingSpeed)
         {
            double adjustedSwingTime = maxSpeedDimensionless / maxSwingSpeed;
            footstepData.setTimings(adjustedSwingTime, transferTime);
            simulationTime += adjustedSwingTime;
         }
         else
            simulationTime += swingTime;

         footsteps.add(footstepData);
      }

      drcSimulationTestHelper.send(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);

      Point3d rootJointPosition = new Point3d(2.81, 0.0, 0.79);
      Vector3d epsilon = new Vector3d(0.05, 0.05, 0.05);
      Point3d min = new Point3d(rootJointPosition);
      Point3d max = new Point3d(rootJointPosition);
      min.sub(epsilon);
      max.add(epsilon);

      if (LOCAL_MODE)
      {
         ThreadTools.sleepForever();
      }
      else
      {
         drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(new BoundingBox3d(min, max));
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
      simulationTestingParameters = null;
   }
}
