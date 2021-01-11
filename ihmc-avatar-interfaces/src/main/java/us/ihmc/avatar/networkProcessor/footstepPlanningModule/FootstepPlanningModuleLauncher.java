package us.ihmc.avatar.networkProcessor.footstepPlanningModule;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlannerRequestedAction;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.FootstepPlannerOccupancyMapAssembler;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerOccupancyMap;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

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
      SwingPlannerParametersBasics swingPlannerParameters = robotModel.getSwingPlannerParameters();

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      SideDependentList<ConvexPolygon2D> footPolygons = createFootPolygons(robotModel);

      return new FootstepPlanningModule(moduleName,
                                        visibilityGraphsParameters,
                                        footstepPlannerParameters,
                                        swingPlannerParameters,
                                        walkingControllerParameters,
                                        footPolygons);
   }

   /**
    * Creates a FootstepPlanningModule and creates ROS 2 subscribers and publishers on a new ros node
    */
   public static FootstepPlanningModule createModule(DRCRobotModel robotModel, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      LogTools.info("Starting footstep planning module in ROS 2 {} mode", pubSubImplementation.name());
      ROS2Node ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, "footstep_planner");
      return createModule(ros2Node, robotModel);
   }

   /**
    * Creates a FootstepPlannerModule object and creates ROS 2 subscribers and publishers
    */
   public static FootstepPlanningModule createModule(ROS2Node ros2Node, DRCRobotModel robotModel)
   {
      FootstepPlanningModule footstepPlanningModule = createModule(robotModel);
      footstepPlanningModule.registerRosNode(ros2Node);
      String name = footstepPlanningModule.getName();
      ROS2Topic inputTopic = ROS2Tools.FOOTSTEP_PLANNER.withRobot(name).withInput();
      ROS2Topic outputTopic = ROS2Tools.FOOTSTEP_PLANNER.withRobot(name).withOutput();

      AtomicBoolean generateLog = new AtomicBoolean();

      createParametersCallbacks(ros2Node, footstepPlanningModule, inputTopic);
      createRequestCallback(ros2Node, footstepPlanningModule, inputTopic, generateLog);
      createStatusPublisher(ros2Node, footstepPlanningModule, outputTopic);
      createOccupancyGridCallback(ros2Node, footstepPlanningModule, outputTopic);
      createPlannerActionCallback(ros2Node, footstepPlanningModule, inputTopic, outputTopic);
      createLoggerCallback(footstepPlanningModule, generateLog);

      return footstepPlanningModule;
   }

   private static void createParametersCallbacks(ROS2Node ros2Node,
                                                 FootstepPlanningModule footstepPlanningModule,
                                                 ROS2Topic inputTopic)
   {
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepPlannerParametersPacket.class, inputTopic, s ->
      {
         if (!footstepPlanningModule.isPlanning())
            footstepPlanningModule.getFootstepPlannerParameters().set(s.readNextData());
      });
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, VisibilityGraphsParametersPacket.class, inputTopic, s ->
      {
         if (!footstepPlanningModule.isPlanning())
            footstepPlanningModule.getVisibilityGraphParameters().set(s.takeNextData());
      });
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, SwingPlannerParametersPacket.class, inputTopic, s ->
      {
         if (!footstepPlanningModule.isPlanning())
            footstepPlanningModule.getSwingPlannerParameters().set(s.takeNextData());
      });
   }

   private static void createRequestCallback(ROS2Node ros2Node,
                                             FootstepPlanningModule footstepPlanningModule,
                                             ROS2Topic inputTopic,
                                             AtomicBoolean generateLog)
   {
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepPlanningRequestPacket.class, inputTopic, s ->
      {
         FootstepPlannerRequest request = new FootstepPlannerRequest();
         FootstepPlanningRequestPacket requestPacket = s.takeNextData();
         request.setFromPacket(requestPacket);
         generateLog.set(requestPacket.getGenerateLog());
         new Thread(() -> footstepPlanningModule.handleRequest(request), "FootstepPlanningRequestHandler").start();
      });
   }

   private static void createStatusPublisher(ROS2Node ros2Node, FootstepPlanningModule footstepPlanningModule, ROS2Topic outputTopic)
   {
      IHMCROS2Publisher<FootstepPlanningToolboxOutputStatus> resultPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                                                                  FootstepPlanningToolboxOutputStatus.class,
                                                                                                                  outputTopic);

      footstepPlanningModule.addStatusCallback(output ->
                                               {
                                                  FootstepPlanningToolboxOutputStatus outputStatus = new FootstepPlanningToolboxOutputStatus();
                                                  output.setPacket(outputStatus);
                                                  resultPublisher.publish(outputStatus);
                                               });
   }

   private static void createOccupancyGridCallback(ROS2Node ros2Node,
                                                   FootstepPlanningModule footstepPlanningModule,
                                                   ROS2Topic outputTopic)
   {
      IHMCROS2Publisher<FootstepPlannerOccupancyMapMessage> occupancyMapPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                                                                       FootstepPlannerOccupancyMapMessage.class,
                                                                                                                       outputTopic);
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

   private static void createPlannerActionCallback(ROS2Node ros2Node,
                                                   FootstepPlanningModule footstepPlanningModule,
                                                   ROS2Topic inputTopic,
                                                   ROS2Topic outputTopic)
   {
      IHMCROS2Publisher<FootstepPlannerParametersPacket> parametersPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                                                                  FootstepPlannerParametersPacket.class,
                                                                                                                  outputTopic);

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

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepPlannerActionMessage.class, inputTopic, s ->
      {
         s.takeNextData(footstepPlannerActionMessage, null);
         new Thread(callback, "FootstepPlannerActionCallback").start();
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

   public static SideDependentList<ConvexPolygon2D> createFootPolygons(DRCRobotModel robotModel)
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
