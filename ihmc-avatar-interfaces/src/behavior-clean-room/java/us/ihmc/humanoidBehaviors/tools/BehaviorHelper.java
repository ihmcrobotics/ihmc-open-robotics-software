package us.ihmc.humanoidBehaviors.tools;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidBehaviors.RemoteREAInterface;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerInterface;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerResult;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.TopicListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.ActivationReference;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.tools.thread.TypedNotification;

import java.util.concurrent.atomic.AtomicReference;

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
   private final ManagedMessager messager;
   private final ManagedROS2Node ros2Node;

   private final RemoteFootstepPlannerInterface remoteFootstepPlannerInterface;
   private final RemoteREAInterface remoteREAInterface;

   public BehaviorHelper(DRCRobotModel robotModel, Messager messager, Ros2Node ros2Node)
   {
      this.robotModel = robotModel;
      this.messager = new ManagedMessager(messager);
      this.ros2Node = new ManagedROS2Node(ros2Node);

      // TODO: Remove all this construction until needed

      // TODO: Create enable/disable support for these

      remoteFootstepPlannerInterface = new RemoteFootstepPlannerInterface(ros2Node, robotModel, messager); // planner toolbox
      remoteREAInterface = new RemoteREAInterface(ros2Node); // REA toolbox

      // TODO: Extract UI comms class

      // TODO: Make accessors to classes; interface?
   }

   // Construction-only methods:

   // TODO

   public RemoteHumanoidRobotInterface createRobotInterface()
   {
      return new RemoteHumanoidRobotInterface(ros2Node, robotModel);
   }

   // Robot Command Methods:


   // Robot Action Callback and Polling Methods:

   // RobotEnvironmentAwareness Methods:

   public PlanarRegionsList getLatestPlanarRegionList()
   {
      return remoteREAInterface.getLatestPlanarRegionList();
   }

   public PlanarRegionsListMessage getLatestPlanarRegionListMessage()
   {
      return remoteREAInterface.getLatestPlanarRegionListMessage();
   }

   public void clearREA()
   {
      remoteREAInterface.clearREA();
   }

   // Planning Methods:

   public TypedNotification<RemoteFootstepPlannerResult> requestPlan(FramePose3DReadOnly start, FramePose3DReadOnly goal,
                                                                     PlanarRegionsList planarRegionsList)
   {
      PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
      return requestPlan(start, goal, planarRegionsListMessage);
   }

   public TypedNotification<RemoteFootstepPlannerResult> requestPlan(FramePose3DReadOnly start, FramePose3DReadOnly goal,
                                                                     PlanarRegionsListMessage planarRegionsListMessage)
   {
      return remoteFootstepPlannerInterface.requestPlan(start, goal, planarRegionsListMessage);
   }

   public void abortPlanning()
   {
      remoteFootstepPlannerInterface.abortPlanning();
   }

   // UI Communication Methods:

   public <T> void publishToUI(Topic<T> topic, T message)
   {
      messager.submitMessage(topic, message);
   }

   public ActivationReference<Boolean> createBooleanActivationReference(Topic<Boolean> topic)
   {
      return messager.createBooleanActivationReference(topic);
   }

   public <T> void createUICallback(Topic<T> topic, TopicListener<T> listener)
   {
      messager.registerTopicListener(topic, listener);
   }

   public <T> AtomicReference<T> createUIInput(Topic<T> topic, T initialValue)
   {
      return messager.createInput(topic, initialValue);
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

   // TODO: Extract to behavior manager in general?
   public void setCommunicationCallbacksEnabled(boolean enabled)
   {
      ros2Node.setEnabled(enabled);
      messager.setEnabled(enabled);
   }

   public Messager getMessager()
   {
      return messager;
   }
}
