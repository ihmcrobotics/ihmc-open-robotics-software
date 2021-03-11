package us.ihmc.humanoidBehaviors.tools;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import std_msgs.msg.dds.Empty;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.avatar.ros2.ROS2ControllerPublisherMap;
import us.ihmc.avatar.sensors.realsense.MapsenseTools;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.*;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.SwingPlanningModule;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerInterface;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.communication.ros2.ManagedROS2Node;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.communication.ros2.ROS2TypelessInput;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.Messager;
import us.ihmc.messager.TopicListener;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.robotEnvironmentAwareness.updaters.GPUPlanarRegionUpdater;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.*;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.tools.thread.ActivationReference;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Class for entry methods for developing robot behaviors. The idea is to have this be the one-stop
 * shopping location for everything one might want to do when creating a robot behavior. It should
 * hide all of the network traffic. The methods should be a useful reminder to a behavior developer
 * all of the things that one can do with the robots. This class will likely get too large
 * eventually and need to be refactored into several classes. But until that time comes it should
 * contain everything for interacting with: the robot actions (taking steps and achieving poses),
 * robot sensing (reference frames, etc.), REA (getting planar regions), footstep planning, etc. At
 * first we'll make this so that all of the things are created, even if you don't need them. But
 * later, we'll make it so that they are created as needed. The main goal is to simplify and make
 * clean the Behaviors themselves. The public interface of this class should be a joy to work with.
 * The internals and all the things it relies on might be a nightmare, but the public API should not
 * be.
 *
 * Open question: Trust vs. power vs. safety for/from behavior authors
 *
 * Robot:
 * - Command-only
 * - Status-only
 * - Interactive (footstep completion, hand trajectory completion, etc.)
 *
 * UI Communication.
 *
 * Toolbox comms:
 * - REA Input/Output
 * - Footstep planner
 *
 * Helper tools (threading, etc.)
 */
public class BehaviorHelper
{
   private final DRCRobotModel robotModel;
   private final ManagedMessager managedMessager;
   private final RosNodeInterface ros1Node;
   private final ManagedROS2Node managedROS2Node;
   private final ROS2PublisherMap ros2PublisherMap;
   private final ROS2ControllerPublisherMap ros2ControllerPublisherMap;
   private RemoteHumanoidRobotInterface robot;
   private RemoteFootstepPlannerInterface footstepPlannerToolbox;
   private RemoteREAInterface rea;
   private RemoteEnvironmentMapInterface environmentMap;
   private VisibilityGraphPathPlanner bodyPathPlanner;
   private FootstepPlanningModule footstepPlanner;
   private StatusLogger statusLogger;

   public BehaviorHelper(DRCRobotModel robotModel, Messager messager, RosNodeInterface ros1Node, ROS2NodeInterface ros2Node)
   {
      this(robotModel, messager, ros1Node, ros2Node, true);
   }

   public BehaviorHelper(DRCRobotModel robotModel,
                         Messager messager,
                         RosNodeInterface ros1Node,
                         ROS2NodeInterface ros2Node,
                         boolean commsEnabledToStart)
   {
      this.robotModel = robotModel;
      managedMessager = new ManagedMessager(messager);
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
         footstepPlannerToolbox = new RemoteFootstepPlannerInterface(managedROS2Node, robotModel, managedMessager);
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

   public StatusLogger getOrCreateStatusLogger()
   {
      if (statusLogger == null)
         statusLogger = new StatusLogger(this::publish);
      return statusLogger;
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

   public void subscribeToPlanarRegionsViaCallback(String topic, Consumer<PlanarRegionsList> callback)
   {
      boolean daemon = true;
      int queueSize = 1;
      ResettableExceptionHandlingExecutorService executorService
            = MissingThreadTools.newSingleThreadExecutor("ROS1PlanarRegionsSubscriber", daemon, queueSize);
      GPUPlanarRegionUpdater gpuPlanarRegionUpdater = new GPUPlanarRegionUpdater();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
      subscribeViaCallback(ROS2Tools.getRobotConfigurationDataTopic(robotModel.getSimpleRobotName()), robotConfigurationDataBuffer::update);
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      RigidBodyTransform transform = robotModel.getSensorInformation().getSteppingCameraTransform();
      ReferenceFrame baseFrame = robotModel.getSensorInformation().getSteppingCameraFrame(referenceFrames);
      ReferenceFrame sensorFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("l515", baseFrame, transform);
      MapsenseTools.createROS1Callback(topic, ros1Node, rawGPUPlanarRegionList ->
      {
         executorService.clearQueueAndExecute(() ->
         {
            robotConfigurationDataBuffer.updateFullRobotModel(false, rawGPUPlanarRegionList.getHeader().getStamp().totalNsecs(), fullRobotModel, null);
            try
            {
               referenceFrames.updateFrames();
            }
            catch (NotARotationMatrixException e)
            {
               LogTools.error(e.getMessage());
            }
            PlanarRegionsList planarRegionsList = gpuPlanarRegionUpdater.generatePlanarRegions(rawGPUPlanarRegionList);
            try
            {
               planarRegionsList.applyTransform(MapsenseTools.getTransformFromCameraToWorld());
               planarRegionsList.applyTransform(sensorFrame.getTransformToWorldFrame());
            }
            catch (NotARotationMatrixException e)
            {
               LogTools.error(e.getMessage());
            }
            callback.accept(planarRegionsList);
         });
      });
   }

   public <T> void subscribeViaCallback(ROS2Topic<T> topic, Consumer<T> callback)
   {
      new IHMCROS2Callback<>(managedROS2Node, topic, callback);
   }

   public void subscribeViaCallback(ROS2Topic<Empty> topic, Runnable callback)
   {
      new IHMCROS2Callback<>(managedROS2Node, topic, message -> callback.run());
   }

   // TODO: Move to remote robot interface?
   public <T> void subscribeToControllerViaCallback(Class<T> messageClass, Consumer<T> callback)
   {
      new IHMCROS2Callback<>(managedROS2Node, ControllerAPIDefinition.getTopic(messageClass, robotModel.getSimpleRobotName()), callback);
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
   }

   // UI Communication Methods:
   // Extract into class?

   public <T> void publish(Topic<T> topic, T message)
   {
      managedMessager.submitMessage(topic, message);
   }

   public void publish(Topic<Object> topic)
   {
      managedMessager.submitMessage(topic, new Object());
   }

   public ActivationReference<Boolean> subscribeViaActivationReference(Topic<Boolean> topic)
   {
      return managedMessager.createBooleanActivationReference(topic);
   }

   public <T> void subscribeViaCallback(Topic<T> topic, TopicListener<T> listener)
   {
      managedMessager.registerTopicListener(topic, listener);
   }

   public <T> AtomicReference<T> subscribeViaReference(Topic<T> topic, T initialValue)
   {
      return managedMessager.createInput(topic, initialValue);
   }

   public Notification subscribeTypelessViaNotification(Topic<Object> topic)
   {
      Notification notification = new Notification();
      subscribeViaCallback(topic, object -> notification.set());
      return notification;
   }

   public <T extends K, K> TypedNotification<K> subscribeViaNotification(Topic<T> topic)
   {
      TypedNotification<K> typedNotification = new TypedNotification<>();
      subscribeViaCallback(topic, message -> typedNotification.set(message));
      return typedNotification;
   }

   // ROS 2 Methods:

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
      managedMessager.setEnabled(enabled);
   }

   public Messager getMessager()
   {
      return managedMessager;
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
