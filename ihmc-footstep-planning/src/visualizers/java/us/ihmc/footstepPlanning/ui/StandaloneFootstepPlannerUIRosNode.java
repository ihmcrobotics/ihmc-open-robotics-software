package us.ihmc.footstepPlanning.ui;

import com.sun.javafx.application.PlatformImpl;
import controller_msgs.msg.dds.*;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.ros2.RealtimeRos2Node;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI.ComputePathTopic;
import static us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI.PlanningResultTopic;

public class StandaloneFootstepPlannerUIRosNode
{
   // TODO make a local thing of planar regions

   private static final boolean debug = false;
   private final RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ihmc_footstep_planner_ui");
   private Optional<PlanarRegionsList> planarRegionsList = Optional.empty();

   private final StandaloneFootstepPlannerUILauncher launcher;
   private final StandaloneFootstepPlannerUI ui;
   private final JavaFXMessager messager;

   private final String robotName;

   private final AtomicReference<FootstepPlan> footstepPlanReference;
   private final AtomicReference<FootstepPlanningResult> plannerResultReference;

   private IHMCRealtimeROS2Publisher<FootstepPlanningToolboxOutputStatus> footstepPlanningRequestPublisher;

   private int planId = 0;

   public StandaloneFootstepPlannerUIRosNode(String robotName)
   {
      this(robotName, false);
   }

   public StandaloneFootstepPlannerUIRosNode(String robotName, boolean visualize)
   {
      this.robotName = robotName;

      launcher = new StandaloneFootstepPlannerUILauncher(visualize);

      PlatformImpl.startup(() -> {
         Platform.runLater(new Runnable()
         {
            @Override
            public void run()
            {
               try
               {
                  launcher.start(new Stage());
               }
               catch (Exception e)
               {
                  e.printStackTrace();
               }
            }
         });
      });
      PlatformImpl.setImplicitExit(false);

      while (launcher.getUI() == null)
         ThreadTools.sleep(100);

      ui = launcher.getUI();

      messager = ui.getMessager();

      footstepPlanReference = messager.createInput(FootstepPlannerUserInterfaceAPI.FootstepPlanTopic, null);
      plannerResultReference = messager.createInput(FootstepPlannerUserInterfaceAPI.PlanningResultTopic, null);

      footstepPlanningRequestPublisher = ROS2Tools
            .createPublisher(ros2Node, FootstepPlanningToolboxOutputStatus.class, getOutputSubscriberTopicNameGenerator());

      ROS2Tools.createCallbackSubscription(ros2Node, PlanarRegionsListMessage.class, REACommunicationProperties.publisherTopicNameGenerator,
                                           s -> handlePlanarRegionsMessage(s.takeNextData()));

      messager.registerTopicListener(FootstepPlannerUserInterfaceAPI.PlanningResultTopic, request -> sendResultingPlan());

      ros2Node.spin();
   }

   public void destroy() throws Exception
   {
      launcher.stop();
      ros2Node.destroy();
   }

   private void sendResultingPlan()
   {
      if (plannerResultReference.get().validForExecution())
      {
         int tick = 0;
         while (footstepPlanReference.get() == null)
         {
            ThreadTools.sleep(10);
            tick++;
            if (tick > 1000)
               throw new RuntimeException("Waited for too many ticks the expected valid footstep plan.");
         }

         footstepPlanningRequestPublisher.publish(packResult(footstepPlanReference.getAndSet(null), plannerResultReference.getAndSet(null)));
      }
   }

   private void handlePlanarRegionsMessage(PlanarRegionsListMessage message)
   {
      if (message == null)
      {
         this.planarRegionsList = Optional.empty();
      }
      else
      {
         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(message);
         this.planarRegionsList = Optional.of(planarRegionsList);
      }
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlanarRegionDataTopic, this.planarRegionsList.get());

   }

   private FootstepPlanningToolboxOutputStatus packResult(FootstepPlan footstepPlan, FootstepPlanningResult status)
   {
      if (debug)
      {
         PrintTools.info("Finished planning. Result: " + status);
      }

      FootstepPlanningToolboxOutputStatus result = new FootstepPlanningToolboxOutputStatus();
      if (footstepPlan == null)
      {
         result.getFootstepDataList().set(new FootstepDataListMessage());
      }
      else
      {
         result.getFootstepDataList().set(createFootstepDataListFromPlan(footstepPlan, 0.0, 0.0, ExecutionMode.OVERRIDE));
      }

      result.setPlanId(planId++);
      result.setFootstepPlanningResult(status.toByte());
      planarRegionsList.ifPresent(regions -> result.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regions)));

      return result;
   }

   public StandaloneFootstepPlannerUI getUI()
   {
      return ui;
   }

   private ROS2Tools.MessageTopicNameGenerator getOutputSubscriberTopicNameGenerator()
   {
      return getOutputSubscriberTopicNameGenerator(robotName);
   }

   private static ROS2Tools.MessageTopicNameGenerator getOutputSubscriberTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.OUTPUT);
   }

   public static FootstepDataListMessage createFootstepDataListFromPlan(FootstepPlan footstepPlan, double swingTime, double transferTime,
                                                                        ExecutionMode executionMode)
   {
      if (footstepPlan == null)
         return null;

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingDuration(swingTime);
      footstepDataListMessage.setDefaultTransferDuration(transferTime);

      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         SimpleFootstep footstep = footstepPlan.getFootstep(i);

         FramePose3D footstepPose = new FramePose3D();
         footstep.getSoleFramePose(footstepPose);
         Point3D location = new Point3D(footstepPose.getPosition());
         Quaternion orientation = new Quaternion(footstepPose.getOrientation());

         FootstepDataMessage footstepData = createFootstepDataMessage(footstep.getRobotSide(), location, orientation);

         if (footstep.hasFoothold())
         {
            ConvexPolygon2D foothold = new ConvexPolygon2D();
            footstep.getFoothold(foothold);

            if (foothold.getNumberOfVertices() != 4)
               ConvexPolygonTools.limitVerticesConservative(foothold, 4);

            ArrayList<Point2D> contactPoints = new ArrayList<>();
            for (int contactPointIdx = 0; contactPointIdx < 4; contactPointIdx++)
               contactPoints.add(new Point2D(foothold.getVertex(contactPointIdx)));
            packPredictedContactPoints(contactPoints, footstepData);
         }

         footstepDataListMessage.getFootstepDataList().add().set(footstepData);
      }

      return footstepDataListMessage;
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3D location, Quaternion orientation)
   {
      return createFootstepDataMessage(robotSide, location, orientation, null);
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3D location, Quaternion orientation,
                                                               ArrayList<Point2D> predictedContactPoints)
   {
      return createFootstepDataMessage(robotSide, location, orientation, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public static FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, Point3D location, Quaternion orientation,
                                                               List<Point2D> predictedContactPoints, TrajectoryType trajectoryType, double swingHeight)
   {
      FootstepDataMessage message = new FootstepDataMessage();
      message.setRobotSide(robotSide.toByte());
      message.getLocation().set(location);
      message.getOrientation().set(orientation);
      packPredictedContactPoints(predictedContactPoints, message);
      message.setTrajectoryType(trajectoryType.toByte());
      message.setSwingHeight(swingHeight);
      return message;
   }

   public static void packPredictedContactPoints(List<? extends Point2DReadOnly> contactPoints, FootstepDataMessage message)
   {
      if (contactPoints == null)
         return;

      message.getPredictedContactPoints2d().clear();

      for (int i = 0; i < contactPoints.size(); i++)
      {
         message.getPredictedContactPoints2d().add().set(contactPoints.get(i), 0.0);
      }
   }

   public static void main(String[] args)
   {
      new StandaloneFootstepPlannerUIRosNode("", true);
   }

}
