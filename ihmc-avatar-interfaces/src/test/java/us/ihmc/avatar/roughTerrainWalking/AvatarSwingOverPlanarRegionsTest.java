package us.ihmc.avatar.roughTerrainWalking;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsStandaloneVisualizer;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.LittleWallsWithIncreasingHeightPlanarRegionEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class AvatarSwingOverPlanarRegionsTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SCS2AvatarTestingSimulation simulationTestHelper;

   private static final boolean LOCAL_MODE = false;

   @Test
   public void testSwingOverPlanarRegions()
   {
      double swingTime = 0.6;
      double transferTime = 0.25;
      double stepLength = 0.35;
      double stepWidth = 0.14;
      double maxSwingSpeed = 1.0;
      int steps = 10;

      LittleWallsWithIncreasingHeightPlanarRegionEnvironment environment = new LittleWallsWithIncreasingHeightPlanarRegionEnvironment();
      PlanarRegionsList planarRegionsList = environment.getPlanarRegionsList();

      DRCRobotModel robotModel = getRobotModel();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(robotModel, environment, simulationTestingParameters);
      simulationTestHelper.start();
      simulationTestHelper.setCameraPosition(8.0, -8.0, 5.0);
      simulationTestHelper.setCameraFocusPosition(1.5, 0.0, 0.8);

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();

      SwingOverPlanarRegionsStandaloneVisualizer swingOverPlanarRegionsVisualizer = null;
      SwingOverPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;
      ConvexPolygon2D footPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(contactPointParameters.getFootContactPoints().get(RobotSide.LEFT)));
      if (LOCAL_MODE)
      {
         // FIXME Didn't migrate the viz to SCS2
         //         swingOverPlanarRegionsVisualizer = new SwingOverPlanarRegionsStandaloneVisualizer(simulationTestHelper,
         //                                                                                           registry,
         //                                                                                           yoGraphicsListRegistry,
         //                                                                                           walkingControllerParameters,
         //                                                                                           footPolygon);
         //         swingOverPlanarRegionsTrajectoryExpander = swingOverPlanarRegionsVisualizer.getSwingOverPlanarRegionsTrajectoryExpander();
      }
      else
      {
         swingOverPlanarRegionsTrajectoryExpander = new SwingOverPlanarRegionsTrajectoryExpander(walkingControllerParameters, registry, yoGraphicsListRegistry);
      }

      simulationTestHelper.getRootRegistry().addChild(registry);
      simulationTestHelper.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      simulationTestHelper.simulateNow(0.5);

      FramePose3D stanceFootPose = new FramePose3D();
      FramePose3D swingStartPose = new FramePose3D();
      FramePose3D swingEndPose = new FramePose3D();

      stanceFootPose.getPosition().set(0.0, -stepWidth, 0.0);
      swingEndPose.getPosition().set(0.0, stepWidth, 0.0);

      FootstepDataListMessage footsteps = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      double simulationTime = transferTime * steps + 1.0;
      for (int i = 1; i <= steps; i++)
      {
         RobotSide robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide.negateIfRightSide(stepWidth);
         double footstepX = stepLength * i;
         Point3D location = new Point3D(footstepX, footstepY, 0.0);
         Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);

         swingStartPose.set(stanceFootPose);
         stanceFootPose.set(swingEndPose);
         swingEndPose.getPosition().set(footstepX, footstepY, 0.0);
         double maxSpeedDimensionless = Double.NaN;

         if (LOCAL_MODE)
         {
            maxSpeedDimensionless = swingOverPlanarRegionsVisualizer.expandTrajectoryOverPlanarRegions(stanceFootPose,
                                                                                                       swingStartPose,
                                                                                                       swingEndPose,
                                                                                                       planarRegionsList);
         }
         else
         {
            maxSpeedDimensionless = swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(stanceFootPose,
                                                                                                               swingStartPose,
                                                                                                               swingEndPose,
                                                                                                               planarRegionsList);
         }

         LogTools.info("Step " + i + ": " + swingOverPlanarRegionsTrajectoryExpander.getStatus());
         LogTools.info("Foot: " + robotSide + "  X: " + footstepX + "  Y: " + footstepY);

         footstepData.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
         Point3D waypointOne = new Point3D(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(0));
         Point3D waypointTwo = new Point3D(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(1));
         MessageTools.copyData(new Point3D[] {waypointOne, waypointTwo}, footstepData.getCustomPositionWaypoints());

         double maxSpeed = maxSpeedDimensionless / swingTime;
         if (maxSpeed > maxSwingSpeed)
         {
            double adjustedSwingTime = maxSpeedDimensionless / maxSwingSpeed;
            footstepData.setSwingDuration(adjustedSwingTime);
            footstepData.setTransferDuration(transferTime);
            simulationTime += adjustedSwingTime;
         }
         else
            simulationTime += swingTime;

         footsteps.getFootstepDataList().add().set(footstepData);
      }

      simulationTestHelper.publishToController(footsteps);
      simulationTestHelper.simulateNow(simulationTime);

      Point3D rootJointPosition = new Point3D(3.25, 0.0, 0.83);
      Vector3D epsilon = new Vector3D(0.05, 0.05, 0.10);
      Point3D min = new Point3D(rootJointPosition);
      Point3D max = new Point3D(rootJointPosition);
      min.sub(epsilon);
      max.add(epsilon);

      if (LOCAL_MODE)
      {
         ThreadTools.sleepForever();
      }
      else
      {
         simulationTestHelper.assertRobotsRootJointIsInBoundingBox(new BoundingBox3D(min, max));
      }
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
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

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      simulationTestingParameters = null;
   }
}
