package us.ihmc.atlas.behaviors;

import static org.junit.jupiter.api.Assertions.*;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedHumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.SimulatedREAModule;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.PlannerTestEnvironments;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.OcclusionHandlingPathPlanner;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.PathOrientationCalculator;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

import java.util.ArrayList;
import java.util.List;

public class AtlasCorridorNavigationTest
{
   private static boolean VISUALIZE = Boolean.parseBoolean(System.getProperty("visualize")); // To visualize, pass -Dvisualize=true

   @BeforeAll
   static public void beforeAll()
   {
      if (VISUALIZE) JavaFXApplicationCreator.createAJavaFXApplication();
   }

   @AfterEach
   public void afterEach()
   {
      if (VISUALIZE) ThreadTools.sleepForever();
   }

   @Test
   public void testAtlasMakesItToGoalInTrickyCorridor()
   {
      // start_position 0.0 0.0 0.0
      // goal_position 6.0 0.0 0.0
      // vis_graph_status test
      // step_planner_status dev
      // a_star_timeout 10.0
      // vis_graph_with_a_star_timeout 45.0
      // requires_occlusion false

      new Thread(() -> {
         LogTools.info("Creating simulate REA module");
         new SimulatedREAModule(PlannerTestEnvironments.getTrickCorridor(), createRobotModel(), PubSubImplementation.INTRAPROCESS).start();
      }).start();

      new Thread(() -> {
         LogTools.info("Creating simulation");
         boolean createYoVariableServer = false;
         HumanoidKinematicsSimulation.createForAutomatedTest(createRobotModel(), createYoVariableServer);
      }).start();

      new Thread(() -> {
         LogTools.info("Creating footstep toolbox");
         new MultiStageFootstepPlanningModule(createRobotModel(), null, false, PubSubImplementation.INTRAPROCESS);
      }).start();


      if (VISUALIZE)
      {
//         LogTools.info("Creating behavior user interface");
//         new BehaviorUI(messager, createRobotModel(), DomainFactory.PubSubImplementation.INTRAPROCESS);


      }

      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.INTRAPROCESS, "test_node");

      // create map subscriber
      ROS2Input<PlanarRegionsListMessage> mapRegionsInput = new ROS2Input<>(ros2Node, PlanarRegionsListMessage.class, null, ROS2Tools.REA);

      // subscribe to robot pose
      RemoteSyncedHumanoidRobotState humanoidRobotState = new RemoteSyncedHumanoidRobotState(createRobotModel(), ros2Node);

      boolean fullyExpandVisibilityGraph = false;
      Point3D start = new Point3D();
      Point3D goal = new Point3D(6.0, 0.0, 0.0);
      FramePose3D robotPose = new FramePose3D();
      robotPose.setToZero(humanoidRobotState.pollHumanoidRobotState().getMidFeetZUpFrame());
      robotPose.changeFrame(ReferenceFrame.getWorldFrame());
      List<Point3DReadOnly> pathPoints = null;
      List<? extends Pose3DReadOnly> path = null;

      while (robotPose.getPosition().distance(goal) > 0.5)
      {
         VisibilityGraphsParametersBasics parameters = new DefaultVisibilityGraphParameters();
         NavigableRegionsManager manager = new NavigableRegionsManager(parameters, null, new ObstacleAvoidanceProcessor(parameters));
         OcclusionHandlingPathPlanner occlusionHandlingPathPlanner = new OcclusionHandlingPathPlanner(manager);
         PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(parameters);
         manager.setPlanarRegions(PlanarRegionMessageConverter.convertToPlanarRegionsList(mapRegionsInput.getLatest()).getPlanarRegionsAsList());
         robotPose.setToZero(humanoidRobotState.pollHumanoidRobotState().getMidFeetZUpFrame());
         robotPose.changeFrame(ReferenceFrame.getWorldFrame());
         pathPoints = occlusionHandlingPathPlanner.calculateBodyPath(robotPose.getPosition(), goal, fullyExpandVisibilityGraph);
         path = orientationCalculator.computePosesFromPath(pathPoints, manager.getVisibilityMapSolution(), new Quaternion(), new Quaternion());

         // make robot walk with path


         // wait for robot to finish walking


         robotPose.setToZero(humanoidRobotState.pollHumanoidRobotState().getMidFeetZUpFrame());
         robotPose.changeFrame(ReferenceFrame.getWorldFrame());
      }
   }

   private AtlasRobotModel createRobotModel()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, true);
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, simulationContactPoints);
   }
}
