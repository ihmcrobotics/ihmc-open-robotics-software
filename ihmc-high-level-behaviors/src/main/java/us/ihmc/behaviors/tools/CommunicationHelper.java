package us.ihmc.behaviors.tools;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import std_msgs.msg.dds.Empty;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.avatar.ros2.ROS2ControllerPublisherMap;
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
import us.ihmc.communication.ros2.ManagedROS2Node;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.communication.ros2.ROS2TypelessInput;
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
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class CommunicationHelper
{
   protected final DRCRobotModel robotModel;
   protected final RosNodeInterface ros1Node;
   protected final ManagedROS2Node managedROS2Node;
   protected final ROS2PublisherMap ros2PublisherMap;
   protected final ROS2ControllerPublisherMap ros2ControllerPublisherMap;
   private RemoteHumanoidRobotInterface robot;
   private RemoteFootstepPlannerInterface footstepPlannerToolbox;
   private RemoteREAInterface rea;
   private RemoteEnvironmentMapInterface environmentMap;
   private VisibilityGraphPathPlanner bodyPathPlanner;
   private FootstepPlanningModule footstepPlanner;

   public CommunicationHelper(DRCRobotModel robotModel, RosNodeInterface ros1Node, ROS2NodeInterface ros2Node)
   {
      this(robotModel, ros1Node, ros2Node, true);
   }

   public CommunicationHelper(DRCRobotModel robotModel,
                              RosNodeInterface ros1Node,
                              ROS2NodeInterface ros2Node,
                              boolean commsEnabledToStart)
   {
      this.robotModel = robotModel;
      this.ros1Node = ros1Node;
      managedROS2Node = new ManagedROS2Node(ros2Node);

      ros2PublisherMap = new ROS2PublisherMap(managedROS2Node);
      ros2ControllerPublisherMap = new ROS2ControllerPublisherMap(ros2Node, robotModel.getSimpleRobotName(), ros2PublisherMap);

      setCommunicationCallbacksEnabled(commsEnabledToStart);
   }

   // Construction-only methods:
   // These not safe yet. "Create" needs to happen at construction or not at all. Maybe doesn't matter right now.

   public RemoteHumanoidRobotInterface getOrCreateRobotInterface()
   {
      if (robot == null)
         robot = new RemoteHumanoidRobotInterface(managedROS2Node, robotModel);
      return robot;
   }

   public RemoteFootstepPlannerInterface getOrCreateFootstepPlannerToolboxInterface()
   {
      if (footstepPlannerToolbox == null)
         footstepPlannerToolbox = new RemoteFootstepPlannerInterface(managedROS2Node, robotModel, null);
      return footstepPlannerToolbox; // planner toolbox
   }

   public RemoteREAInterface getOrCreateREAInterface()
   {
      if (rea == null)
         rea = new RemoteREAInterface(managedROS2Node);
      return rea; // REA toolbox
   }

   public RemoteEnvironmentMapInterface getOrCreateEnvironmentMapInterface()
   {
      if (environmentMap == null)
         environmentMap = new RemoteEnvironmentMapInterface(managedROS2Node);
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

   public RemoteSyncedRobotModel newSyncedRobot()
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
      return robotModel.newRobotLowLevelMessenger(managedROS2Node);
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
                                     walkingControllerParameters);
   }

   public DelayFixedPlanarRegionsSubscription subscribeToPlanarRegionsViaCallback(String topic, Consumer<PlanarRegionsList> callback)
   {
      return MapsenseTools.subscribeToPlanarRegionsWithDelayCompensation(managedROS2Node, robotModel, topic, callback);
   }

   public <T> void subscribeViaCallback(ROS2Topic<T> topic, Consumer<T> callback)
   {
      ROS2Tools.createCallbackSubscription2(managedROS2Node, topic, callback);
   }

   public void subscribeViaCallback(ROS2Topic<Empty> topic, Runnable callback)
   {
      ROS2Tools.createCallbackSubscription2(managedROS2Node, topic, message -> callback.run());
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
      ROS2Input<PlanarRegionsListMessage> input = new ROS2Input<>(managedROS2Node, topic.getType(), topic);
      return () -> PlanarRegionMessageConverter.convertToPlanarRegionsList(input.getLatest());
   }

   public <T> ROS2Input<T> subscribe(ROS2Topic<T> topic)
   {
      return new ROS2Input<>(managedROS2Node, topic.getType(), topic);
   }

   public ROS2TypelessInput subscribeTypeless(ROS2Topic<Empty> topic)
   {
      return new ROS2TypelessInput(managedROS2Node, topic);
   }

   public Notification subscribeViaNotification(ROS2Topic<Empty> topic)
   {
      Notification notification = new Notification();
      new ROS2Callback<>(managedROS2Node, Empty.class, topic, message -> notification.set());
      return notification;
   }

   public <T> void publish(ROS2Topic<T> topic, T message)
   {
      ros2PublisherMap.publish(topic, message);
   }

   public void publish(ROS2Topic<Pose3D> topic, Pose3D message)
   {
      ros2PublisherMap.publish(topic, message);
   }

   public void publish(ROS2Topic<Empty> topic)
   {
      ros2PublisherMap.publish(topic);
   }

   public void publishToController(Object message)
   {
      ros2ControllerPublisherMap.publish(message);
   }// ROS 2 Methods:

   public Notification subscribeToWalkingCompletedViaNotification()
   {
      Notification notification = new Notification();
      subscribeToControllerViaCallback(WalkingStatusMessage.class, walkingStatusMessage -> {
         if (walkingStatusMessage.getWalkingStatus() == WalkingStatusMessage.COMPLETED)
         {
            notification.set();
         }
      });
      return notification;
   }

   // Thread and Schedule Methods:
   // TODO: Track and auto start/stop threads?

   public PausablePeriodicThread createPausablePeriodicThread(Class<?> clazz, double period, Runnable runnable)
   {
      return createPausablePeriodicThread(clazz.getSimpleName(), period, 0, runnable);
   }

   public PausablePeriodicThread createPausablePeriodicThread(Class<?> clazz, double period, int crashesBeforeGivingUp, Runnable runnable)
   {
      return createPausablePeriodicThread(clazz.getSimpleName(), period, crashesBeforeGivingUp, runnable);
   }

   public PausablePeriodicThread createPausablePeriodicThread(String name, double period, int crashesBeforeGivingUp, Runnable runnable)
   {
      return new PausablePeriodicThread(name, period, crashesBeforeGivingUp, runnable);
   }

   // Behavior Helper Stuff:

   // Let behaviors manage or manage for them?
   // Split into finer granularity -- publishers and subscribers?
   public void setCommunicationCallbacksEnabled(boolean enabled)
   {
      managedROS2Node.setEnabled(enabled);
   }

   public RosNodeInterface getROS1Node()
   {
      return ros1Node;
   }

   public ROS2NodeInterface getROS2Node()
   {
      return managedROS2Node;
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
