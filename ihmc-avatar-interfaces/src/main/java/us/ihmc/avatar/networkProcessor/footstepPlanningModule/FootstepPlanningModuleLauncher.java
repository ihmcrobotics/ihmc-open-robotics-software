package us.ihmc.avatar.networkProcessor.footstepPlanningModule;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.footstepPlanning.AdaptiveSwingTrajectoryCalculator;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.RosBasedPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.parameters.AdaptiveSwingParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.util.ArrayList;

import static us.ihmc.footstepPlanning.FootstepPlannerStatus.*;
import static us.ihmc.footstepPlanning.FootstepPlannerStatus.IDLE;

public class FootstepPlanningModuleLauncher
{
   public static FootstepPlanningModule createModule(DRCRobotModel robotModel)
   {
      String moduleName = robotModel.getSimpleRobotName();
      FootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      VisibilityGraphsParametersBasics visibilityGraphsParameters = robotModel.getVisibilityGraphsParameters();
      SideDependentList<ConvexPolygon2D> footPolygons = createFootPolygons(robotModel);

      return new FootstepPlanningModule(moduleName, footstepPlannerParameters, visibilityGraphsParameters, footPolygons);
   }

   public static FootstepPlanningModule createModule(DRCRobotModel robotModel, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      return createModule(robotModel, pubSubImplementation, null);
   }

   public static FootstepPlanningModule createModule(DRCRobotModel robotModel, DomainFactory.PubSubImplementation pubSubImplementation, AdaptiveSwingParameters swingParameters)
   {
      FootstepPlanningModule footstepPlanningModule = createModule(robotModel);
      Ros2Node ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, "footstep_planner");
      footstepPlanningModule.registerRosNode(ros2Node);

      String name = footstepPlanningModule.getName();
      ROS2Tools.MessageTopicNameGenerator subscriberTopicNameGenerator = ROS2Tools.getTopicNameGenerator(name, ROS2Tools.FOOTSTEP_PLANNER_MODULE, ROS2Tools.ROS2TopicQualifier.INPUT);
      ROS2Tools.MessageTopicNameGenerator publisherTopicNameGenerator = ROS2Tools.getTopicNameGenerator(name, ROS2Tools.FOOTSTEP_PLANNER_MODULE, ROS2Tools.ROS2TopicQualifier.OUTPUT);

      // Parameters callback
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlannerParametersPacket.class, subscriberTopicNameGenerator, s ->
      {
         if (!footstepPlanningModule.isPlanning())
            footstepPlanningModule.getFootstepPlannerParameters().set(s.readNextData());
      });
      ROS2Tools.createCallbackSubscription(ros2Node, VisibilityGraphsParametersPacket.class, subscriberTopicNameGenerator, s ->
      {
         if (!footstepPlanningModule.isPlanning())
            footstepPlanningModule.getVisibilityGraphParameters().set(s.takeNextData());
      });

      // Planner request callback
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningRequestPacket.class, subscriberTopicNameGenerator, s ->
      {
         FootstepPlannerRequest request = new FootstepPlannerRequest();
         request.setFromPacket(s.takeNextData());
         new Thread(() -> footstepPlanningModule.handleRequest(request)).start();
      });

      // Body path plan publisher
      IHMCROS2Publisher<BodyPathPlanMessage> bodyPathPlanPublisher = ROS2Tools.createPublisher(ros2Node, BodyPathPlanMessage.class, publisherTopicNameGenerator);
      footstepPlanningModule.addBodyPathPlanCallback(bodyPathPlanPublisher::publish);

      // Status publisher
      IHMCROS2Publisher<FootstepPlanningToolboxOutputStatus> resultPublisher = ROS2Tools.createPublisher(ros2Node,
                                                                                                         FootstepPlanningToolboxOutputStatus.class,
                                                                                                         publisherTopicNameGenerator);

      AdaptiveSwingTrajectoryCalculator swingParameterCalculator =
            swingParameters == null ? null : new AdaptiveSwingTrajectoryCalculator(swingParameters, robotModel.getWalkingControllerParameters());
      footstepPlanningModule.addStatusCallback(output ->
                                               {
                                                  FootstepPlanningToolboxOutputStatus outputStatus = new FootstepPlanningToolboxOutputStatus();
                                                  output.setPacket(outputStatus);
                                                  if (swingParameterCalculator != null)
                                                  {
                                                     swingParameterCalculator.setPlanarRegionsList(footstepPlanningModule.getRequest().getPlanarRegionsList());
                                                     swingParameterCalculator.setSwingParameters(footstepPlanningModule.getRequest().getStanceFootPose(),
                                                                                                 outputStatus.getFootstepDataList());
                                                  }
                                                  resultPublisher.publish(outputStatus);
                                               });

      // planner listener
      long updateFrequency = 500;
      IHMCROS2Publisher<FootstepNodeDataListMessage> plannerNodeDataPublisher = ROS2Tools.createPublisher(ros2Node,
                                                                                                          FootstepNodeDataListMessage.class,
                                                                                                          publisherTopicNameGenerator);
      IHMCROS2Publisher<FootstepPlannerOccupancyMapMessage> occupancyMapPublisher = ROS2Tools.createPublisher(ros2Node,
                                                                                                              FootstepPlannerOccupancyMapMessage.class,
                                                                                                              publisherTopicNameGenerator);
      RosBasedPlannerListener plannerListener = new RosBasedPlannerListener(plannerNodeDataPublisher, occupancyMapPublisher, footstepPlanningModule.getSnapper(), updateFrequency);
      footstepPlanningModule.getChecker().addPlannerListener(plannerListener);

      footstepPlanningModule.addRequestCallback(request -> plannerListener.reset());
      footstepPlanningModule.addIterationCallback(iterationData ->
                           {
                              if (iterationData.getParentNode() == null)
                                 return;
                              for (int i = 0; i < iterationData.getValidChildNodes().size(); i++)
                                 plannerListener.addNode(iterationData.getValidChildNodes().get(i), iterationData.getParentNode());
                              for (int i = 0; i < iterationData.getInvalidChildNodes().size(); i++)
                                 plannerListener.addNode(iterationData.getInvalidChildNodes().get(i), iterationData.getParentNode());
                              plannerListener.tickAndUpdate();
                           });

      // status publisher
      IHMCROS2Publisher<FootstepPlannerStatusMessage> statusPublisher = ROS2Tools.createPublisher(ros2Node,
                                                                                                  FootstepPlannerStatusMessage.class,
                                                                                                  publisherTopicNameGenerator);
      footstepPlanningModule.addRequestCallback(request ->
                                                {
                                                   FootstepPlannerStatusMessage statusMessage = new FootstepPlannerStatusMessage();
                                                   statusMessage.setFootstepPlannerStatus((request.getPlanBodyPath() ? PLANNING_PATH : PLANNING_STEPS).toByte());
                                                   statusPublisher.publish(statusMessage);
                                                });
      footstepPlanningModule.addBodyPathPlanCallback(bodyPathPlanMessage ->
                                                     {
                                                        FootstepPlannerStatusMessage statusMessage = new FootstepPlannerStatusMessage();
                                                        boolean planningSteps = FootstepPlanningResult.fromByte(bodyPathPlanMessage.getFootstepPlanningResult())
                                                                                                      .validForExecution();
                                                        statusMessage.setFootstepPlannerStatus((planningSteps ? PLANNING_STEPS : IDLE).toByte());
                                                        statusPublisher.publish(statusMessage);
                                                     });
      footstepPlanningModule.addStatusCallback(output ->
                                               {
                                                  boolean plannerTerminated = output.getResult() != FootstepPlanningResult.SOLUTION_DOES_NOT_REACH_GOAL;
                                                  if (plannerTerminated)
                                                  {
                                                     FootstepPlannerStatusMessage statusMessage = new FootstepPlannerStatusMessage();
                                                     statusMessage.setFootstepPlannerStatus((IDLE).toByte());
                                                     statusPublisher.publish(statusMessage);
                                                  }
                                               });

      // cancel planning request
      ROS2Tools.createCallbackSubscription(ros2Node, ToolboxStateMessage.class, subscriberTopicNameGenerator, s ->
      {
         if(ToolboxState.fromByte(s.takeNextData().getRequestedToolboxState()) == ToolboxState.SLEEP)
            footstepPlanningModule.halt();
      });

      return footstepPlanningModule;
   }

   private static SideDependentList<ConvexPolygon2D> createFootPolygons(DRCRobotModel robotModel)
   {
      if (robotModel.getContactPointParameters() == null)
      {
         return PlannerTools.createDefaultFootPolygons();
      }

      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      return new SideDependentList<>(side ->
                                     {
                                        ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(side);
                                        return new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints));
                                     });
   }
}
