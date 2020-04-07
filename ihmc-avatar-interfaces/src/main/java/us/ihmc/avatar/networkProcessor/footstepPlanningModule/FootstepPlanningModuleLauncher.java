package us.ihmc.avatar.networkProcessor.footstepPlanningModule;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.footstepPlanning.AdaptiveSwingTrajectoryCalculator;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlannerRequestedAction;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.FootstepPlannerOccupancyMapAssembler;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerOccupancyMap;
import us.ihmc.footstepPlanning.graphSearch.parameters.AdaptiveSwingParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.time.DayOfWeek;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

public class FootstepPlanningModuleLauncher
{
   private static final String LOG_DIRECTORY_ENVIRONMENT_VARIABLE = "IHMC_FOOTSTEP_PLANNER_LOG_DIR";
   private static final String LOG_DIRECTORY;

   static
   {
      String requestedLogDirectory = System.getenv(LOG_DIRECTORY_ENVIRONMENT_VARIABLE);
      LOG_DIRECTORY = requestedLogDirectory == null ? FootstepPlannerLogger.getDefaultLogsDirectory() : requestedLogDirectory;
   }

   /**
    * Creates a FootstepPlanningModule object given a DRCRobotModel
    */
   public static FootstepPlanningModule createModule(DRCRobotModel robotModel)
   {
      String moduleName = robotModel.getSimpleRobotName();
      FootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      VisibilityGraphsParametersBasics visibilityGraphsParameters = robotModel.getVisibilityGraphsParameters();
      SideDependentList<ConvexPolygon2D> footPolygons = createFootPolygons(robotModel);

      return new FootstepPlanningModule(moduleName, footstepPlannerParameters, visibilityGraphsParameters, footPolygons);
   }

   /**
    * Creates a FootstepPlanningModule and creates ROS 2 subscribers and publishers on a new ros node
    */
   public static FootstepPlanningModule createModule(DRCRobotModel robotModel, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      return createModule(robotModel, pubSubImplementation, null);
   }

   /**
    * Creates a FootstepPlanningModule and creates ROS 2 subscribers and publishers on a new ros node.
    * Sets swing trajectories with the given AdaptiveSwingParameters
    */
   public static FootstepPlanningModule createModule(DRCRobotModel robotModel,
                                                     DomainFactory.PubSubImplementation pubSubImplementation,
                                                     AdaptiveSwingParameters swingParameters)
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, "footstep_planner");
      return createModule(ros2Node, robotModel, swingParameters);
   }

   /**
    * Creates a FootstepPlanningModule and creates ROS 2 subscribers and publishers on the given ros node.
    * Sets swing trajectories with the given AdaptiveSwingParameters
    */
   public static FootstepPlanningModule createModule(Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      return createModule(ros2Node, robotModel, null);
   }

   /**
    * Creates a FootstepPlannerModule object and creates ROS 2 subscribers and publishers
    */
   public static FootstepPlanningModule createModule(Ros2Node ros2Node, DRCRobotModel robotModel, AdaptiveSwingParameters swingParameters)
   {
      FootstepPlanningModule footstepPlanningModule = createModule(robotModel);
      footstepPlanningModule.registerRosNode(ros2Node);
      String name = footstepPlanningModule.getName();
      ROS2Tools.MessageTopicNameGenerator subscriberTopicNameGenerator = ROS2Tools.getTopicNameGenerator(name,
                                                                                                         ROS2Tools.FOOTSTEP_PLANNER_MODULE,
                                                                                                         ROS2Tools.ROS2TopicQualifier.INPUT);
      ROS2Tools.MessageTopicNameGenerator publisherTopicNameGenerator = ROS2Tools.getTopicNameGenerator(name,
                                                                                                        ROS2Tools.FOOTSTEP_PLANNER_MODULE,
                                                                                                        ROS2Tools.ROS2TopicQualifier.OUTPUT);

      AtomicBoolean generateLog = new AtomicBoolean();

      createParametersCallbacks(ros2Node, footstepPlanningModule, subscriberTopicNameGenerator);
      createRequestCallback(ros2Node, footstepPlanningModule, subscriberTopicNameGenerator, generateLog);
      createStatusPublisher(ros2Node, robotModel, swingParameters, footstepPlanningModule, publisherTopicNameGenerator);
      createOccupancyGridCallback(ros2Node, footstepPlanningModule, publisherTopicNameGenerator);
      createPlannerActionCallback(ros2Node, footstepPlanningModule, subscriberTopicNameGenerator, publisherTopicNameGenerator);
      createLoggerCallback(footstepPlanningModule, generateLog);

      return footstepPlanningModule;
   }

   private static void createParametersCallbacks(Ros2Node ros2Node,
                                                 FootstepPlanningModule footstepPlanningModule,
                                                 MessageTopicNameGenerator subscriberTopicNameGenerator)
   {
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
   }

   private static void createRequestCallback(Ros2Node ros2Node,
                                             FootstepPlanningModule footstepPlanningModule,
                                             MessageTopicNameGenerator subscriberTopicNameGenerator,
                                             AtomicBoolean generateLog)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningRequestPacket.class, subscriberTopicNameGenerator, s ->
      {
         FootstepPlannerRequest request = new FootstepPlannerRequest();
         FootstepPlanningRequestPacket requestPacket = s.takeNextData();
         request.setFromPacket(requestPacket);
         generateLog.set(requestPacket.getGenerateLog());
         new Thread(() -> footstepPlanningModule.handleRequest(request)).start();
      });
   }

   private static void createStatusPublisher(Ros2Node ros2Node,
                                             DRCRobotModel robotModel,
                                             AdaptiveSwingParameters swingParameters,
                                             FootstepPlanningModule footstepPlanningModule,
                                             MessageTopicNameGenerator publisherTopicNameGenerator)
   {
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
                                                     RobotSide initialStanceSide = footstepPlanningModule.getRequest().getRequestedInitialStanceSide();
                                                     Pose3D initialStancePose = footstepPlanningModule.getRequest().getStartFootPoses().get(initialStanceSide);
                                                     swingParameterCalculator.setSwingParameters(initialStancePose, outputStatus.getFootstepDataList());
                                                  }
                                                  resultPublisher.publish(outputStatus);
                                               });
   }


   private static void createOccupancyGridCallback(Ros2Node ros2Node,
                                                   FootstepPlanningModule footstepPlanningModule,
                                                   MessageTopicNameGenerator publisherTopicNameGenerator)
   {
      IHMCROS2Publisher<FootstepPlannerOccupancyMapMessage> occupancyMapPublisher = ROS2Tools.createPublisher(ros2Node,
                                                                                                              FootstepPlannerOccupancyMapMessage.class,
                                                                                                              publisherTopicNameGenerator);
      FootstepPlannerOccupancyMapAssembler occupancyMapAssembler = new FootstepPlannerOccupancyMapAssembler();
      footstepPlanningModule.addRequestCallback(request -> occupancyMapAssembler.reset());
      footstepPlanningModule.addIterationCallback(occupancyMapAssembler);
      footstepPlanningModule.addStatusCallback(status ->
                                               {
                                                  PlannerOccupancyMap occupancyMap = occupancyMapAssembler.getOccupancyMap();
                                                  if (!occupancyMap.isEmpty())
                                                  {
                                                     occupancyMapPublisher.publish(occupancyMap.getAsMessage());
                                                     occupancyMapAssembler.getOccupancyMap().clear();
                                                  }
                                               });
   }

   private static void createPlannerActionCallback(Ros2Node ros2Node,
                                                   FootstepPlanningModule footstepPlanningModule,
                                                   MessageTopicNameGenerator subscriberTopicNameGenerator,
                                                   MessageTopicNameGenerator publisherTopicNameGenerator)
   {
      IHMCROS2Publisher<FootstepPlannerParametersPacket> parametersPublisher = ROS2Tools.createPublisher(ros2Node,
                                                                                               FootstepPlannerParametersPacket.class,
                                                                                               publisherTopicNameGenerator);

      FootstepPlannerActionMessage footstepPlannerActionMessage = new FootstepPlannerActionMessage();
      FootstepPlannerParametersPacket footstepPlannerParametersPacket = new FootstepPlannerParametersPacket();

      Runnable callback = () ->
      {
         FootstepPlannerRequestedAction requestedAction = FootstepPlannerRequestedAction.fromByte(footstepPlannerActionMessage.getRequestedAction());
         if (requestedAction == FootstepPlannerRequestedAction.HALT)
         {
            footstepPlanningModule.halt();
         }
         else if (requestedAction == FootstepPlannerRequestedAction.PUBLISH_PARAMETERS)
         {
            FootstepPlannerMessageTools.copyParametersToPacket(footstepPlannerParametersPacket, footstepPlanningModule.getFootstepPlannerParameters());
            parametersPublisher.publish(footstepPlannerParametersPacket);
         }
      };

      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlannerActionMessage.class, subscriberTopicNameGenerator, s ->
      {
         s.takeNextData(footstepPlannerActionMessage, null);
         new Thread(callback).start();
      });
   }

   private static void createLoggerCallback(FootstepPlanningModule footstepPlanningModule, AtomicBoolean generateLog)
   {
      FootstepPlannerLogger logger = new FootstepPlannerLogger(footstepPlanningModule);
      footstepPlanningModule.addStatusCallback(status ->
                                               {
                                                  if (status.getFootstepPlanningResult() != null && status.getFootstepPlanningResult().terminalResult()
                                                      && generateLog.get())
                                                     logger.logSession(LOG_DIRECTORY);
                                               });
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
