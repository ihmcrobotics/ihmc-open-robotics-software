package us.ihmc.footstepPlanning.ui;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeRos2Node;

import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

public class StandaloneFootstepPlannerUIMessageConverter
{
   private final RealtimeRos2Node ros2Node;

   private final JavaFXMessager messager;
   private final String robotName;

   private IHMCRealtimeROS2Publisher<FootstepPlanningToolboxOutputStatus> outputStatusPublisher;

   private Optional<PlanarRegionsList> planarRegionsList = Optional.empty();

   private final AtomicReference<FootstepPlanningResult> resultReference;
   private final AtomicReference<FootstepPlan> footstepPlanReference;
   private final AtomicReference<Integer> plannerRequestIdReference;
   private final AtomicReference<Integer> sequenceIdReference;



   public static StandaloneFootstepPlannerUIMessageConverter createRemoteConverter(JavaFXMessager messager, String robotName)
   {
      return createConverter(messager, robotName, DomainFactory.PubSubImplementation.FAST_RTPS);
   }

   public static StandaloneFootstepPlannerUIMessageConverter createIntraprocessConverter(JavaFXMessager messager, String robotName)
   {
      return createConverter(messager, robotName, DomainFactory.PubSubImplementation.INTRAPROCESS);
   }

   public static StandaloneFootstepPlannerUIMessageConverter createConverter(JavaFXMessager messager, String robotName, DomainFactory.PubSubImplementation implementation)
   {
      RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(implementation, "ihmc_footstep_planner_ui");
      return new StandaloneFootstepPlannerUIMessageConverter(ros2Node, messager, robotName);
   }

   public StandaloneFootstepPlannerUIMessageConverter(RealtimeRos2Node ros2Node, JavaFXMessager messager, String robotName)
   {
      this.messager = messager;
      this.robotName = robotName;
      this.ros2Node = ros2Node;

      resultReference = messager.createInput(FootstepPlannerUserInterfaceAPI.PlanningResultTopic);
      footstepPlanReference = messager.createInput(FootstepPlannerUserInterfaceAPI.FootstepPlanTopic);
      plannerRequestIdReference = messager.createInput(FootstepPlannerUserInterfaceAPI.PlannerRequestIdTopic);
      sequenceIdReference = messager.createInput(FootstepPlannerUserInterfaceAPI.SequenceIdTopic);

      registerPubSubs(ros2Node);

      ros2Node.spin();
   }

   public void destroy()
   {
      ros2Node.destroy();
   }

   private void registerPubSubs(RealtimeRos2Node ros2Node)
   {
      /* subscribers */
      // we want to listen to the incoming request
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningRequestPacket.class, getPlanningToolboxSubscriberNameGenerator(),
                                           s -> processFootstepPlanningRequestPacket(s.takeNextData()));

      // publishers
      outputStatusPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlanningToolboxOutputStatus.class, getPlanningToolboxPublisherNameGenerator());

      messager.registerTopicListener(FootstepPlannerUserInterfaceAPI.FootstepPlanTopic, request -> publishResultingPlan());
   }

   private void processFootstepPlanningRequestPacket(FootstepPlanningRequestPacket packet)
   {
      PlanarRegionsListMessage planarRegionsListMessage = packet.getPlanarRegionsListMessage();
      if (planarRegionsListMessage == null)
      {
         this.planarRegionsList = Optional.empty();
      }
      else
      {
         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
         this.planarRegionsList = Optional.of(planarRegionsList);
      }

      Point3D goalPosition = packet.getGoalPositionInWorld();
      Quaternion goalOrientation = packet.getGoalOrientationInWorld();
      Point3D startPosition = packet.getStanceFootPositionInWorld();
      Quaternion startOrientation = packet.getStanceFootOrientationInWorld();
      FootstepPlannerType plannerType = FootstepPlannerType.fromByte(packet.getRequestedFootstepPlannerType());
      RobotSide initialSupportSide = RobotSide.fromByte(packet.getInitialStanceRobotSide());
      int plannerRequestId = packet.getPlannerRequestId();
      int sequenceId = (int) packet.getSequenceId();

      double timeout = packet.getTimeout();
      double horizonLength = packet.getHorizonLength();

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlanarRegionDataTopic,
                             PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.StartPositionTopic, startPosition);
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.GoalPositionTopic, goalPosition);

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.StartOrientationTopic, startOrientation);
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.GoalOrientationTopic, goalOrientation);

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlannerTypeTopic, plannerType);

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlannerTimeoutTopic, timeout);
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.InitialSupportSideTopic, initialSupportSide);

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlannerRequestIdTopic, plannerRequestId);
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.SequenceIdTopic, sequenceId);

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlannerHorizonLengthTopic, horizonLength);
   }

   private void publishResultingPlan()
   {
      FootstepPlanningToolboxOutputStatus result = new FootstepPlanningToolboxOutputStatus();

      planarRegionsList.ifPresent(regions -> result.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regions)));
      result.setFootstepPlanningResult(resultReference.get().toByte());
      result.setSequenceId(sequenceIdReference.get());
      result.setPlanId(plannerRequestIdReference.get());

      result.getFootstepDataList().set(convertToFootstepDataListMessage(footstepPlanReference.get()));
   }

   private static FootstepDataListMessage convertToFootstepDataListMessage(FootstepPlan footstepPlan)
   {
      if (footstepPlan == null)
      {
         return new FootstepDataListMessage();
      }
      else
      {
         FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();

         for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
         {
            SimpleFootstep footstep = footstepPlan.getFootstep(i);

            FramePose3D footstepPose = new FramePose3D();
            footstep.getSoleFramePose(footstepPose);
            Point3D location = new Point3D(footstepPose.getPosition());
            Quaternion orientation = new Quaternion(footstepPose.getOrientation());

            FootstepDataMessage footstepData = new FootstepDataMessage();
            footstepData.setRobotSide(footstep.getRobotSide().toByte());
            footstepData.getLocation().set(location);
            footstepData.getOrientation().set(orientation);

            if (footstep.hasFoothold())
            {
               ConvexPolygon2D foothold = new ConvexPolygon2D();
               footstep.getFoothold(foothold);

               if (foothold.getNumberOfVertices() != 4)
                  ConvexPolygonTools.limitVerticesConservative(foothold, 4);

               ArrayList<Point2D> contactPoints = new ArrayList<>();
               for (int contactPointIdx = 0; contactPointIdx < 4; contactPointIdx++)
                  contactPoints.add(new Point2D(foothold.getVertex(contactPointIdx)));

               footstepData.getPredictedContactPoints2d().clear();

               for (int j = 0; j < contactPoints.size(); j++)
               {
                  footstepData.getPredictedContactPoints2d().add().set(contactPoints.get(j), 0.0);
               }
            }

            footstepDataListMessage.getFootstepDataList().add().set(footstepData);
         }

         return footstepDataListMessage;
      }
   }


   private ROS2Tools.MessageTopicNameGenerator getPlanningToolboxSubscriberNameGenerator()
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);
   }

   private ROS2Tools.MessageTopicNameGenerator getPlanningToolboxPublisherNameGenerator()
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.OUTPUT);
   }


}
