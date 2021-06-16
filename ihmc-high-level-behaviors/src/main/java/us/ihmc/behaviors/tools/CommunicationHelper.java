package us.ihmc.behaviors.tools;

import controller_msgs.msg.dds.DoorLocationPacket;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import std_msgs.msg.dds.Empty;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.avatar.networkProcessor.objectDetectorToolBox.ObjectDetectorToolboxModule;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.avatar.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.avatar.sensors.realsense.DelayFixedPlanarRegionsSubscription;
import us.ihmc.avatar.sensors.realsense.MapsenseTools;
import us.ihmc.behaviors.tools.footstepPlanner.RemoteFootstepPlannerInterface;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.RemoteREAInterface;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.ros2.*;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.SwingPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * The idea for this class is to provide a flat API for publishing
 * and subscribing where the user doesn't necessarily care which
 * protocol is being used.
 */
public class CommunicationHelper implements ROS2PublishSubscribeAPI, ROS2ControllerPublishSubscribeAPI
{
   protected final DRCRobotModel robotModel;
   protected final ROS2ControllerHelper ros2Helper;

   private RemoteHumanoidRobotInterface robot;
   private RemoteFootstepPlannerInterface footstepPlannerToolbox;
   private RemoteREAInterface rea;
   private RemoteEnvironmentMapInterface environmentMap;
   private VisibilityGraphPathPlanner bodyPathPlanner;
   private FootstepPlanningModule footstepPlanner;

   public CommunicationHelper(DRCRobotModel robotModel, ROS2NodeInterface ros2Node)
   {
      this(robotModel, ros2Node, true);
   }

   public CommunicationHelper(DRCRobotModel robotModel, ROS2NodeInterface ros2Node, boolean commsEnabledToStart)
   {
      this.robotModel = robotModel;
      this.ros2Helper = new ROS2ControllerHelper(ros2Node, robotModel);

      ros2Helper.setCommunicationCallbacksEnabled(commsEnabledToStart);
   }

   // Construction-only methods:
   // These not safe yet. "Create" needs to happen at construction or not at all. Maybe doesn't matter right now.

   public RemoteHumanoidRobotInterface getOrCreateRobotInterface()
   {
      if (robot == null)
         robot = new RemoteHumanoidRobotInterface(ros2Helper.getROS2NodeInterface(), robotModel);
      return robot;
   }

   public RemoteFootstepPlannerInterface getOrCreateFootstepPlannerToolboxInterface()
   {
      if (footstepPlannerToolbox == null)
         footstepPlannerToolbox = new RemoteFootstepPlannerInterface(ros2Helper.getROS2NodeInterface(), robotModel, null);
      return footstepPlannerToolbox; // planner toolbox
   }

   public RemoteREAInterface getOrCreateREAInterface()
   {
      if (rea == null)
         rea = new RemoteREAInterface(ros2Helper.getROS2NodeInterface());
      return rea; // REA toolbox
   }

   public RemoteEnvironmentMapInterface getOrCreateEnvironmentMapInterface()
   {
      if (environmentMap == null)
         environmentMap = new RemoteEnvironmentMapInterface(ros2Helper.getROS2NodeInterface());
      return environmentMap;
   }

   public VisibilityGraphPathPlanner getOrCreateBodyPathPlanner()
   {
      if (bodyPathPlanner == null)
      {
         bodyPathPlanner = newBodyPathPlanner();
      }
      return bodyPathPlanner;
   }

   public ROS2SyncedRobotModel newSyncedRobot()
   {
      return getOrCreateRobotInterface().newSyncedRobot();
   }

   public VisibilityGraphPathPlanner newBodyPathPlanner()
   {
      VisibilityGraphsParametersBasics visibilityGraphsParameters = robotModel.getVisibilityGraphsParameters();
      return new VisibilityGraphPathPlanner(visibilityGraphsParameters, new ObstacleAvoidanceProcessor(visibilityGraphsParameters));
   }

   public RobotLowLevelMessenger newRobotLowLevelMessenger()
   {
      return robotModel.newRobotLowLevelMessenger(ros2Helper.getROS2NodeInterface());
   }

   public FootstepPlanningModule getOrCreateFootstepPlanner()
   {
      if (footstepPlanner == null)
         footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
      return footstepPlanner;
   }

   public SwingPlanningModule createFootstepPlanPostProcessor()
   {
      FootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      SwingPlannerParametersBasics swingPlannerParameters = robotModel.getSwingPlannerParameters();
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      return new SwingPlanningModule(footstepPlannerParameters,
                                     swingPlannerParameters,
                                     walkingControllerParameters,
                                     createFootPolygons());
   }

   public DelayFixedPlanarRegionsSubscription subscribeToPlanarRegionsViaCallback(String topic, Consumer<PlanarRegionsList> callback)
   {
      return MapsenseTools.subscribeToPlanarRegionsWithDelayCompensation(ros2Helper.getROS2NodeInterface(), robotModel, topic, callback);
   }

   @Override
   public <T> void subscribeViaCallback(ROS2Topic<T> topic, Consumer<T> callback)
   {
      ros2Helper.subscribeViaCallback(topic, callback);
   }

   @Override
   public void subscribeViaCallback(ROS2Topic<Empty> topic, Runnable callback)
   {
      ros2Helper.subscribeViaCallback(topic, callback);
   }

   // TODO: Move to remote robot interface?
   public <T> void subscribeToControllerViaCallback(Class<T> messageClass, Consumer<T> callback)
   {
      subscribeViaCallback(ControllerAPIDefinition.getTopic(messageClass, robotModel.getSimpleRobotName()), callback);
   }

   public void subscribeToPlanarRegionsViaCallback(ROS2Topic<PlanarRegionsListMessage> topic, Consumer<PlanarRegionsList> callback)
   {
      subscribeViaCallback(topic, planarRegionsListMessage ->
      {
         callback.accept(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
      });
   }

   public Supplier<PlanarRegionsList> subscribeToPlanarRegionsViaReference(ROS2Topic<PlanarRegionsListMessage> topic)
   {
      ROS2Input<PlanarRegionsListMessage> input = new ROS2Input<>(ros2Helper.getROS2NodeInterface(), topic.getType(), topic);
      return () -> PlanarRegionMessageConverter.convertToPlanarRegionsList(input.getLatest());
   }

   public void subscribeToDoorLocationViaCallback(Consumer<DoorLocationPacket> callback)
   {
      subscribeViaCallback(ObjectDetectorToolboxModule.getOutputTopic(getRobotModel().getSimpleRobotName()).withTypeName(DoorLocationPacket.class), callback);
   }

   @Override
   public <T> void subscribeViaCallback(Function<String, ROS2Topic<T>> topicFunction, Consumer<T> callback)
   {
      ros2Helper.subscribeViaCallback(topicFunction, callback);
   }

   public void subscribeToRobotConfigurationDataViaCallback(Consumer<RobotConfigurationData> callback)
   {
      subscribeViaCallback(ROS2Tools.getRobotConfigurationDataTopic(getRobotModel().getSimpleRobotName()), callback);
   }

   @Override
   public <T> void publish(Function<String, ROS2Topic<T>> topicFunction, T message)
   {
      ros2Helper.publish(topicFunction, message);
   }

   @Override
   public <T> ROS2Input<T> subscribe(ROS2Topic<T> topic)
   {
      return ros2Helper.subscribe(topic);
   }

   @Override
   public ROS2TypelessInput subscribeTypeless(ROS2Topic<Empty> topic)
   {
      return ros2Helper.subscribeTypeless(topic);
   }

   @Override
   public Notification subscribeViaNotification(ROS2Topic<Empty> topic)
   {
      return ros2Helper.subscribeViaNotification(topic);
   }

   @Override
   public <T> void publish(ROS2Topic<T> topic, T message)
   {
      ros2Helper.publish(topic, message);
   }

   @Override
   public void publish(ROS2Topic<Pose3D> topic, Pose3D message)
   {
      ros2Helper.publish(topic, message);
   }

   @Override
   public void publish(ROS2Topic<Empty> topic)
   {
      ros2Helper.publish(topic);
   }

   @Override
   public void publishToController(Object message)
   {
      ros2Helper.publishToController(message);
   }

   @Override
   public Notification subscribeToWalkingCompletedViaNotification()
   {
      return ros2Helper.subscribeToWalkingCompletedViaNotification();
   }

   // Let behaviors manage or manage for them?
   // Split into finer granularity -- publishers and subscribers?
   public void setCommunicationCallbacksEnabled(boolean enabled)
   {
      ros2Helper.setCommunicationCallbacksEnabled(enabled);
   }

   public void destroy()
   {

   }

   public ROS2NodeInterface getROS2Node()
   {
      return ros2Helper.getROS2NodeInterface();
   }

   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   public SideDependentList<ConvexPolygon2D> createFootPolygons()
   {
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(side);
         ConvexPolygon2D scaledFoot = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints));
         footPolygons.set(side, scaledFoot);
      }

      return footPolygons;
   }
}
