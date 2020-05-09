package us.ihmc.quadrupedCommunication.networkProcessing.pawPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersBasics;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxModule;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedNetworkProcessor.footstepPlanningPort;

public class PawPlanningModule extends QuadrupedToolboxModule
{
   private static final int updatePeriodMilliseconds = 50;

   private final PawPlanningController footstepPlanningController;

   public PawPlanningModule(FullQuadrupedRobotModelFactory modelFactory, VisibilityGraphsParametersBasics visibilityGraphsParameters,
                            PawStepPlannerParametersBasics defaultPawPlannerParameters, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                            PointFootSnapperParameters pointFootSnapperParameters, LogModelProvider modelProvider, boolean startYoVariableServer,
                            boolean logYoVariables, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      this(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), visibilityGraphsParameters, defaultPawPlannerParameters,
           defaultXGaitSettings, pointFootSnapperParameters, modelProvider, startYoVariableServer, logYoVariables, pubSubImplementation);
   }

   public PawPlanningModule(String name, FullQuadrupedRobotModel fulRobotModel, VisibilityGraphsParametersBasics visibilityGraphsParameters,
                            PawStepPlannerParametersBasics defaultPawPlannerParameters,
                            QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, PointFootSnapperParameters pointFootSnapperParameters,
                            LogModelProvider modelProvider, boolean startYoVariableServer, boolean logYoVariables,
                            DomainFactory.PubSubImplementation pubSubImplementation)
   {
      super(name, fulRobotModel, modelProvider, startYoVariableServer, new DataServerSettings(logYoVariables, true, footstepPlanningPort,
                                                                                              "FootstepPlanningModule"), updatePeriodMilliseconds,
            pubSubImplementation);


      footstepPlanningController = new PawPlanningController(defaultXGaitSettings, visibilityGraphsParameters, defaultPawPlannerParameters,
                                                             pointFootSnapperParameters, outputManager, robotDataReceiver, registry, yoGraphicsListRegistry,
                                                             updatePeriodMilliseconds);
      new DefaultParameterReader().readParametersInRegistry(registry);
      startYoVariableServer(getClass());
   }

   @Override
   public void registerExtraSubscribers(RealtimeRos2Node realtimeRos2Node)
   {
      // status messages from the controller
      ROS2Topic controllerOutputTopic = ROS2Tools.getQuadrupedControllerOutputTopic(robotName);
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, RobotConfigurationData.class, controllerOutputTopic,
                                           s -> processRobotTimestamp(s.takeNextData().getMonotonicTime()));
//      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, HighLevelStateMessage.class, controllerOutputTopic, s -> footstepPlanningController.setPaused(true));
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, QuadrupedGroundPlaneMessage.class, controllerOutputTopic,
                                           s -> processGroundPlaneMessage(s.takeNextData()));

      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, QuadrupedSupportPlanarRegionParametersMessage.class,
                                                    ROS2Tools.QUADRUPED_SUPPORT_REGION_PUBLISHER.withRobot(robotName)
                                                              .withInput(),
                                           s -> processSupportRegionParameters(s.takeNextData()));

      // inputs to this module
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, PawStepPlanningRequestPacket.class, getInputTopic(),
                                           s -> processPawPlanningRequest(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, QuadrupedXGaitSettingsPacket.class, getInputTopic(),
                                           s -> processXGaitSettingsPacket(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, PawStepPlannerParametersPacket.class, getInputTopic(),
                                           s -> processFootstepPlannerParametersPacket(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, VisibilityGraphsParametersPacket.class, getInputTopic(),
                                           s -> processVisibilityGraphParametersPacket(s.takeNextData()));
   }

   private void processRobotTimestamp(long timestamp)
   {
      if (footstepPlanningController != null)
         footstepPlanningController.processRobotTimestamp(timestamp);
   }

   private void processFootstepPlannerParametersPacket(PawStepPlannerParametersPacket packet)
   {
      if (footstepPlanningController != null)
         footstepPlanningController.processFootstepPlannerParametersPacket(packet);
   }

   private void processVisibilityGraphParametersPacket(VisibilityGraphsParametersPacket packet)
   {
      if (footstepPlanningController != null)
         footstepPlanningController.processVisibilityGraphParametersPacket(packet);
   }

   private void processGroundPlaneMessage(QuadrupedGroundPlaneMessage message)
   {
      if (footstepPlanningController != null)
         footstepPlanningController.processGroundPlaneMessage(message);
   }

   private void processSupportRegionParameters(QuadrupedSupportPlanarRegionParametersMessage message)
   {
      if (footstepPlanningController != null)
         footstepPlanningController.processSupportRegionParameters(message);
   }

   private void processPawPlanningRequest(PawStepPlanningRequestPacket packet)
   {
      if (footstepPlanningController != null)
         footstepPlanningController.processPawPlanningRequest(packet);
   }

   private void processXGaitSettingsPacket(QuadrupedXGaitSettingsPacket packet)
   {
      if (footstepPlanningController != null)
         footstepPlanningController.processXGaitSettingsPacket(packet);
   }

   @Override
   public QuadrupedToolboxController getToolboxController()
   {
      return footstepPlanningController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return new ArrayList<>();
   }

   @Override
   public Map<Class<? extends Settable<?>>, ROS2Topic> createMapOfSupportedOutputMessages()
   {
      Map<Class<? extends Settable<?>>, ROS2Topic> messages = new HashMap<>();

      messages.put(PawStepPlanningToolboxOutputStatus.class, getOutputTopic());
      messages.put(QuadrupedBodyOrientationMessage.class, getOutputTopic());
      messages.put(BodyPathPlanMessage.class, getOutputTopic());
      messages.put(PawStepPlannerParametersPacket.class, getOutputTopic());
      messages.put(FootstepPlannerStatusMessage.class, getOutputTopic());

      return messages;
   }

   @Override
   public ROS2Topic getOutputTopic()
   {
      return ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic getInputTopic()
   {
      return ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName).withInput();
   }

   @Override
   public void sleep()
   {
      footstepPlanningController.processPawPlanningRequest(null);
//      footstepPlanningController.setPaused(true);

      super.sleep();
   }


}
