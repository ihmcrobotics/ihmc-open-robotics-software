package us.ihmc.behaviors.tools;

import controller_msgs.msg.dds.RobotConfigurationData;
import org.apache.commons.lang3.tuple.Pair;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import std_msgs.msg.dds.Bool;
import std_msgs.msg.dds.Empty;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.avatar.sensors.realsense.DelayFixedPlanarRegionsSubscription;
import us.ihmc.avatar.sensors.realsense.MapsenseTools;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.SwingPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.SwapReference;
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
public class CommunicationHelper implements ROS2ControllerPublishSubscribeAPI
{
   protected final DRCRobotModel robotModel;
   protected final ROS2ControllerHelper ros2Helper;

   private RemoteHumanoidRobotInterface robot;
   private FootstepPlanningModule footstepPlanner;
   private RobotLowLevelMessenger lowLevelMessenger;

   public CommunicationHelper(DRCRobotModel robotModel, ROS2Node ros2Node)
   {
      this.robotModel = robotModel;
      this.ros2Helper = new ROS2ControllerHelper(ros2Node, robotModel);
   }

   public ROS2ControllerHelper getControllerHelper()
   {
      return ros2Helper;
   }

   // Construction-only methods:
   // These not safe yet. "Create" needs to happen at construction or not at all. Maybe doesn't matter right now.

   public RemoteHumanoidRobotInterface getOrCreateRobotInterface()
   {
      if (robot == null)
         robot = new RemoteHumanoidRobotInterface(ros2Helper.getROS2Node(), robotModel);
      return robot;
   }


   public ROS2SyncedRobotModel newSyncedRobot()
   {
      return getOrCreateRobotInterface().newSyncedRobot();
   }

   public ROS2SyncedRobotModel newSyncedRobot(boolean enforceUniqueReferenceFrames)
   {
      return getOrCreateRobotInterface().newSyncedRobot(enforceUniqueReferenceFrames);
   }

   public RobotLowLevelMessenger getOrCreateRobotLowLevelMessenger()
   {
      if (lowLevelMessenger == null)
         lowLevelMessenger = robotModel.newRobotLowLevelMessenger(ros2Helper.getROS2Node());

      return lowLevelMessenger;
   }

   public FootstepPlanningModule getOrCreateFootstepPlanner()
   {
      if (footstepPlanner == null)
         footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
      return footstepPlanner;
   }

   public SwingPlanningModule createFootstepPlanPostProcessor()
   {
      DefaultFootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      SwingPlannerParametersBasics swingPlannerParameters = robotModel.getSwingPlannerParameters();
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      return new SwingPlanningModule(footstepPlannerParameters,
                                     swingPlannerParameters,
                                     walkingControllerParameters,
                                     createFootPolygons());
   }

   public DelayFixedPlanarRegionsSubscription subscribeToPlanarRegionsViaCallback(String topic, Consumer<Pair<Long, PlanarRegionsList>> callback)
   {
      return MapsenseTools.subscribeToPlanarRegionsWithDelayCompensation(ros2Helper.getROS2Node(), robotModel, topic, callback);
   }

   @Override
   public <T> void subscribeViaCallback(ROS2Topic<T> topic, Consumer<T> callback)
   {
      ros2Helper.subscribeViaCallback(topic, callback);
   }

   @Override
   public <T> void subscribeViaVolatileCallback(ROS2Topic<T> topic, Consumer<T> callback)
   {
      ros2Helper.subscribeViaVolatileCallback(topic, callback);
   }

   @Override
   public <T> SwapReference<T> subscribeViaSwapReference(ROS2Topic<T> topic, Consumer<T> callback)
   {
      return ros2Helper.subscribeViaSwapReference(topic, callback);
   }

   @Override
   public <T> SwapReference<T> subscribeViaSwapReference(ROS2Topic<T> topic, Notification callback)
   {
      return ros2Helper.subscribeViaSwapReference(topic, callback);
   }

   @Override
   public <T> ConcurrentRingBuffer<T> subscribeViaQueue(ROS2Topic<T> topic)
   {
      return ros2Helper.subscribeViaQueue(topic);
   }

   @Override
   public <T> ConcurrentRingBuffer<T> subscribeViaQueue(ROS2Topic<T> topic, int queueSize, Consumer<T> callback)
   {
      return ros2Helper.subscribeViaQueue(topic, queueSize, callback);
   }

   @Override
   public void subscribeViaCallback(ROS2Topic<Empty> topic, Runnable callback)
   {
      ros2Helper.subscribeViaCallback(topic, callback);
   }

   @Override
   public <T> void createPublisher(ROS2Topic<T> topic)
   {
      ros2Helper.createPublisher(topic);
   }

   @Override
   public <T> ROS2Input<T> subscribeToController(Class<T> messageClass)
   {
      return subscribe(HumanoidControllerAPI.getTopic(messageClass, robotModel.getSimpleRobotName()));
   }

   // TODO: Move to remote robot interface?
   @Override
   public <T> void subscribeToControllerViaCallback(Class<T> messageClass, Consumer<T> callback)
   {
      subscribeViaCallback(HumanoidControllerAPI.getTopic(messageClass, robotModel.getSimpleRobotName()), callback);
   }

   @Override
   public <T> void subscribeToControllerViaVolatileCallback(Class<T> messageClass, Consumer<T> callback)
   {
      subscribeViaVolatileCallback(HumanoidControllerAPI.getTopic(messageClass, robotModel.getSimpleRobotName()), callback);
   }

   @Override
   public ROS2Input<RobotConfigurationData> subscribeToRobotConfigurationData()
   {
      return subscribe(StateEstimatorAPI.getRobotConfigurationDataTopic(getRobotName()));
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
      ROS2Input<PlanarRegionsListMessage> input = new ROS2Input<>(ros2Helper.getROS2Node(), topic.getType(), topic);
      return () -> PlanarRegionMessageConverter.convertToPlanarRegionsList(input.getLatest());
   }

   @Override
   public <T> void subscribeViaCallback(Function<String, ROS2Topic<T>> topicFunction, Consumer<T> callback)
   {
      ros2Helper.subscribeViaCallback(topicFunction, callback);
   }

   @Override
   public <T> void subscribeViaVolatileCallback(Function<String, ROS2Topic<T>> topicFunction, Consumer<T> callback)
   {
      ros2Helper.subscribeViaVolatileCallback(topicFunction, callback);
   }

   public void subscribeToRobotConfigurationDataViaCallback(Consumer<RobotConfigurationData> callback)
   {
      subscribeViaCallback(StateEstimatorAPI.getRobotConfigurationDataTopic(getRobotModel().getSimpleRobotName()), callback);
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
   public <T> ROS2Input<T> subscribe(ROS2Topic<T> topic, ROS2Input.MessageFilter<T> messageFilter)
   {
      return ros2Helper.subscribe(topic, messageFilter);
   }

   @Override
   public Notification subscribeViaNotification(ROS2Topic<Empty> topic)
   {
      return ros2Helper.subscribeViaNotification(topic);
   }

   @Override
   public <T> TypedNotification<T> subscribeViaTypedNotification(ROS2Topic<T> topic)
   {
      return ros2Helper.subscribeViaTypedNotification(topic);
   }

   @Override
   public TypedNotification<Boolean> subscribeViaBooleanNotification(ROS2Topic<Bool> topic)
   {
      return ros2Helper.subscribeViaBooleanNotification(topic);
   }

   @Override
   public <T> void publish(ROS2Topic<T> topic, T message)
   {
      ros2Helper.publish(topic, message);
   }

   @Override
   public void publish(ROS2Topic<std_msgs.msg.dds.String> topic, String message)
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
   public void publish(ROS2Topic<Bool> topic, boolean message)
   {
      ros2Helper.publish(topic, message);
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

   public void destroy()
   {

   }

   public ROS2Node getROS2Node()
   {
      return ros2Helper.getROS2Node();
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

   @Override
   public String getRobotName()
   {
      return robotModel.getSimpleRobotName();
   }
}
