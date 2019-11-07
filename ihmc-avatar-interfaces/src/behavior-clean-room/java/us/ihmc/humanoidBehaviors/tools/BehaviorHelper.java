package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidBehaviors.RemoteREAInterface;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerInterface;
import us.ihmc.humanoidBehaviors.tools.ros2.ManagedROS2Node;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.TopicListener;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.ActivationReference;
import us.ihmc.tools.thread.PausablePeriodicThread;

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
   private final ManagedMessager managedMessager;
   private final ManagedROS2Node managedROS2Node;
   private RemoteHumanoidRobotInterface robot;
   private RemoteFootstepPlannerInterface footstepPlannerToolbox;
   private RemoteREAInterface rea;

   public BehaviorHelper(DRCRobotModel robotModel, Messager messager, Ros2Node ros2Node)
   {
      this.robotModel = robotModel;
      managedMessager = new ManagedMessager(messager);
      managedROS2Node = new ManagedROS2Node(ros2Node);

      setCommunicationCallbacksEnabled(false); // should do this?
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

   // UI Communication Methods:
   // Extract into class?

   public <T> void publishToUI(Topic<T> topic, T message)
   {
      managedMessager.submitMessage(topic, message);
   }

   public ActivationReference<Boolean> createBooleanActivationReference(Topic<Boolean> topic)
   {
      return managedMessager.createBooleanActivationReference(topic);
   }

   public <T> void createUICallback(Topic<T> topic, TopicListener<T> listener)
   {
      managedMessager.registerTopicListener(topic, listener);
   }

   public <T> AtomicReference<T> createUIInput(Topic<T> topic, T initialValue)
   {
      return managedMessager.createInput(topic, initialValue);
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

   public Messager getManagedMessager()
   {
      return managedMessager;
   }
}
