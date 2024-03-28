package us.ihmc.avatar.networkProcessor.footstepPlanningModule;

import controller_msgs.msg.dds.*;
import toolbox_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.communication.FootstepPlannerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

public class FootstepPlanningModuleLauncher
{
   private static final String LOG_DIRECTORY_ENVIRONMENT_VARIABLE = "IHMC_FOOTSTEP_PLANNER_LOG_DIR";
   private static final String LOG_DIRECTORY;

   // TODO publish version of ihmc-commons with access to capacity of RecyclingArrayList so that ros message field capacities can be accessed from the field's java object
   private static final int defaultFootstepPlanCapacity = 50;
   private static final int footstepPlanCapacity;

   static
   {
      int footstepListCapacity = defaultFootstepPlanCapacity;

      try
      {
         FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
         IDLSequence.Object<FootstepDataMessage> footstepDataList = footstepDataListMessage.getFootstepDataList();
         Field valuesField = RecyclingArrayList.class.getDeclaredField("values");
         valuesField.setAccessible(true);
         Object[] values = (Object[]) valuesField.get(footstepDataList);
         footstepListCapacity = values.length;
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      footstepPlanCapacity = footstepListCapacity;
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
      String moduleName = robotModel.getSimpleRobotName();

      FootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
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
    * Creates a FootstepPlanningModule and creates ROS 2 subscribers and publishers on a new ros node
    */
   public static FootstepPlanningModule createModule(DRCRobotModel robotModel, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      LogTools.info("Starting footstep planning module in ROS 2 {} mode", pubSubImplementation.name());
      ROS2Node ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, "footstep_planner");
      return createModule(ros2Node, robotModel, true);
   }

   /**
    * Creates a FootstepPlannerModule object and creates ROS 2 subscribers and publishers
    */
   public static FootstepPlanningModule createModule(ROS2NodeInterface ros2Node, DRCRobotModel robotModel)
   {
      return createModule(ros2Node, robotModel, false);
   }

   /**
    * If we don't create the ROS 2 node, then someone else is responsible for disposing it.
    */
   private static FootstepPlanningModule createModule(ROS2NodeInterface ros2Node, DRCRobotModel robotModel, boolean manageROS2Node)
   {
      FootstepPlanningModule footstepPlanningModule = createModule(robotModel);
      footstepPlanningModule.registerRosNode(ros2Node, manageROS2Node);
      String name = footstepPlanningModule.getName();
      ROS2Topic inputTopic = ROS2Tools.FOOTSTEP_PLANNER.withRobot(name).withInput();
      ROS2Topic outputTopic = ROS2Tools.FOOTSTEP_PLANNER.withRobot(name).withOutput();

      AtomicBoolean generateLog = new AtomicBoolean();

      createParametersCallbacks(ros2Node, footstepPlanningModule, inputTopic);
      createRequestCallback(robotModel.getSimpleRobotName(), ros2Node, footstepPlanningModule, inputTopic, generateLog);
      createStatusPublisher(robotModel.getSimpleRobotName(), ros2Node, footstepPlanningModule, outputTopic);
      createPlannerActionCallback(ros2Node, footstepPlanningModule, inputTopic, outputTopic);
      createLoggerCallback(footstepPlanningModule, generateLog);

      return footstepPlanningModule;
   }

   private static void createParametersCallbacks(ROS2NodeInterface ros2Node,
                                                 FootstepPlanningModule footstepPlanningModule,
                                                 ROS2Topic inputTopic)
   {
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepPlannerParametersPacket.class, inputTopic, s ->
      {
         if (!footstepPlanningModule.isPlanning())
            footstepPlanningModule.getFootstepPlannerParameters().set(s.readNextData());
      });
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, SwingPlannerParametersPacket.class, inputTopic, s ->
      {
         if (!footstepPlanningModule.isPlanning())
            footstepPlanningModule.getSwingPlannerParameters().set(s.takeNextData());
      });
   }

   private static void createRequestCallback(String robotName,
                                             ROS2NodeInterface ros2Node,
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

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, SwingPlanningRequestPacket.class, inputTopic, s ->
      {
         SwingPlannerType swingPlannerType = SwingPlannerType.fromByte(s.takeNextData().getRequestedSwingPlanner());
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

   private static void createStatusPublisher(String robotName, ROS2NodeInterface ros2Node, FootstepPlanningModule footstepPlanningModule, ROS2Topic outputTopic)
   {
      ROS2PublisherBasics<FootstepPlanningToolboxOutputStatus> resultPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(
            FootstepPlanningToolboxOutputStatus.class).withTopic(outputTopic));
      ROS2PublisherBasics<FootstepDataListMessage> swingReplanPublisher = ros2Node.createPublisher(FootstepPlannerAPI.swingReplanOutputTopic(robotName));

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

   private static void createPlannerActionCallback(ROS2NodeInterface ros2Node,
                                                   FootstepPlanningModule footstepPlanningModule,
                                                   ROS2Topic inputTopic,
                                                   ROS2Topic outputTopic)
   {
      ROS2PublisherBasics<FootstepPlannerParametersPacket> parametersPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(
            FootstepPlannerParametersPacket.class).withTopic(outputTopic));

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
