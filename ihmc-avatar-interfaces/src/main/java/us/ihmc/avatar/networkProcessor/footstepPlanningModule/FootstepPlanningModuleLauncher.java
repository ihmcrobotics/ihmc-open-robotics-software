package us.ihmc.avatar.networkProcessor.footstepPlanningModule;

import controller_msgs.FootstepDataListMessage;
import toolbox_msgs.FootstepPlannerActionMessage;
import toolbox_msgs.FootstepPlannerParametersPacket;
import toolbox_msgs.FootstepPlanningRequestPacket;
import toolbox_msgs.FootstepPlanningToolboxOutputStatus;
import toolbox_msgs.SwingPlannerParametersPacket;
import toolbox_msgs.SwingPlanningRequestPacket;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.communication.FootstepPlannerAPI;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlannerRequestedAction;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

public class FootstepPlanningModuleLauncher
{
   private static final String LOG_DIRECTORY_ENVIRONMENT_VARIABLE = "IHMC_FOOTSTEP_PLANNER_LOG_DIR";
   private static final String LOG_DIRECTORY;

   private static final int footstepPlanCapacity;

   static
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepPlanCapacity = footstepDataListMessage.getFootstepDataList().capacity();
   }

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
      return createModule(robotModel, "");
   }

   /**
    * Creates a FootstepPlanningModule object given a DRCRobotModel
    */
   public static FootstepPlanningModule createModule(DRCRobotModel robotModel, String suffix)
   {
      String moduleName = robotModel.getSimpleRobotName();
      DefaultFootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters(suffix);

      SwingPlannerParametersBasics swingPlannerParameters = robotModel.getSwingPlannerParameters();
      StepReachabilityData stepReachabilityData = robotModel.getStepReachabilityData();

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      SideDependentList<ConvexPolygon2D> footPolygons = createFootPolygons(robotModel);

      return new FootstepPlanningModule(moduleName,
                                        robotModel.getAStarBodyPathPlannerParameters(),
                                        footstepPlannerParameters,
                                        swingPlannerParameters,
                                        walkingControllerParameters,
                                        footPolygons,
                                        stepReachabilityData);
   }

   /**
    * Creates a FootstepPlannerModule object and creates ROS 2 subscribers and publishers
    */
   public static FootstepPlanningModule createModule(ROS2Node ros2Node, DRCRobotModel robotModel)
   {
      return createModule(ros2Node, robotModel, false);
   }

   /**
    * If we don't create the ROS 2 node, then someone else is responsible for disposing it.
    */
   private static FootstepPlanningModule createModule(ROS2Node ros2Node, DRCRobotModel robotModel, boolean manageROS2Node)
   {
      FootstepPlanningModule footstepPlanningModule = createModule(robotModel);
      footstepPlanningModule.registerRosNode(ros2Node, manageROS2Node);
      String name = footstepPlanningModule.getName();
      ROS2Topic<?> inputTopic = FootstepPlannerAPI.inputTopic(name);
      ROS2Topic<?> outputTopic = FootstepPlannerAPI.outputTopic(name);

      AtomicBoolean generateLog = new AtomicBoolean();

      createParametersCallbacks(ros2Node, footstepPlanningModule, inputTopic);
      createRequestCallback(robotModel.getSimpleRobotName(), ros2Node, footstepPlanningModule, inputTopic, generateLog);
      createStatusPublisher(robotModel.getSimpleRobotName(), ros2Node, footstepPlanningModule, outputTopic);
      createPlannerActionCallback(ros2Node, footstepPlanningModule, inputTopic, outputTopic);
      createLoggerCallback(footstepPlanningModule, generateLog);

      return footstepPlanningModule;
   }

   private static void createParametersCallbacks(ROS2Node ros2Node,
                                                 FootstepPlanningModule footstepPlanningModule,
                                                 ROS2Topic<?> inputTopic)
   {
      ros2Node.createSubscription(inputTopic.withType(FootstepPlannerParametersPacket.class), s ->
      {
         if (!footstepPlanningModule.isPlanning())
            footstepPlanningModule.getFootstepPlannerParameters().set(s.read());
      });
      ros2Node.createSubscription(inputTopic.withType(SwingPlannerParametersPacket.class), s ->
      {
         if (!footstepPlanningModule.isPlanning())
            footstepPlanningModule.getSwingPlannerParameters().set(s.read());
      });
   }

   private static void createRequestCallback(String robotName,
                                             ROS2Node ros2Node,
                                             FootstepPlanningModule footstepPlanningModule,
                                             ROS2Topic<?> inputTopic,
                                             AtomicBoolean generateLog)
   {
      ros2Node.createSubscription(inputTopic.withType(FootstepPlanningRequestPacket.class), s ->
      {
         FootstepPlannerRequest request = new FootstepPlannerRequest();
         FootstepPlanningRequestPacket requestPacket = s.read();
         request.setFromPacket(requestPacket);
         generateLog.set(requestPacket.getGenerateLog());
         new Thread(() -> footstepPlanningModule.handleRequest(request), "FootstepPlanningRequestHandler").start();
      });

      ros2Node.createSubscription(inputTopic.withType(SwingPlanningRequestPacket.class), s ->
      {
         SwingPlannerType swingPlannerType = SwingPlannerType.fromByte(s.read().getRequestedSwingPlanner());
         if (swingPlannerType == SwingPlannerType.NONE)
         {
            LogTools.info("Received swing replanning request with type NONE, ignoring message");
            return;
         }
         else
         {
            LogTools.info("Replanning swing with " + swingPlannerType);
            new Thread(() -> footstepPlanningModule.recomputeSwingTrajectories(swingPlannerType)).start();
         }
      });
   }

   private static void createStatusPublisher(String robotName, ROS2Node ros2Node, FootstepPlanningModule footstepPlanningModule, ROS2Topic outputTopic)
   {
      ROS2Publisher<FootstepPlanningToolboxOutputStatus> resultPublisher = ros2Node.createPublisher(outputTopic.withType(FootstepPlanningToolboxOutputStatus.class));
      ROS2Publisher<FootstepDataListMessage> swingReplanPublisher = ros2Node.createPublisher(FootstepPlannerAPI.swingReplanOutputTopic(robotName));

      footstepPlanningModule.addStatusCallback(output ->
                                               {
                                                  cropPlanToCapacity(output.getFootstepPlan());
                                                  FootstepPlanningToolboxOutputStatus outputStatus = new FootstepPlanningToolboxOutputStatus();
                                                  output.setPacket(outputStatus);
                                                  resultPublisher.publish(outputStatus);
                                               });
      footstepPlanningModule.addSwingReplanStatusCallback(footstepPlan ->
                                                          {
                                                             LogTools.info("Publishing replanned swings");
                                                             FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan, -1.0, -1.0);
                                                             swingReplanPublisher.publish(footstepDataListMessage);
                                                          });
   }

   private static void cropPlanToCapacity(FootstepPlan footstepPlan)
   {
      while (footstepPlan.getNumberOfSteps() > footstepPlanCapacity)
      {
         footstepPlan.remove(footstepPlan.getNumberOfSteps() - 1);
      }
   }

   private static void createPlannerActionCallback(ROS2Node ros2Node,
                                                   FootstepPlanningModule footstepPlanningModule,
                                                   ROS2Topic inputTopic,
                                                   ROS2Topic outputTopic)
   {
      ROS2Publisher<FootstepPlannerParametersPacket> parametersPublisher = ros2Node.createPublisher(outputTopic.withType(FootstepPlannerParametersPacket.class));

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

      ros2Node.createSubscription(((ROS2Topic<?>) inputTopic).withType(FootstepPlannerActionMessage.class), reader ->
      {
         reader.read(footstepPlannerActionMessage);
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
