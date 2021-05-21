package us.ihmc.behaviors.tools;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.BehaviorRegistry;
import us.ihmc.behaviors.tools.interfaces.MessagerPublishSubscribeAPI;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.TopicListener;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.tools.thread.ActivationReference;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.utilities.ros.ROS1Helper;

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
 *
 * TODO: Extract comms helper that does not have the behavior messager as part of it.
 */
public class BehaviorHelper extends CommunicationHelper implements MessagerPublishSubscribeAPI
{
   private final ROS1Helper ros1Helper;
   private final MessagerHelper messagerHelper = new MessagerHelper(BehaviorRegistry.getActiveRegistry().getMessagerAPI());
   private StatusLogger statusLogger;

   // TODO: Considerations for ROS 1, Messager, and YoVariableClient with reconnecting
   public BehaviorHelper(DRCRobotModel robotModel, String ros1NodeName, ROS2NodeInterface ros2Node)
   {
      this(robotModel, ros1NodeName, ros2Node, true);
   }

   public BehaviorHelper(DRCRobotModel robotModel,
                         String ros1NodeName,
                         ROS2NodeInterface ros2Node,
                         boolean commsEnabledToStart)
   {
      super(robotModel, ros2Node, commsEnabledToStart);
      this.ros1Helper = new ROS1Helper(ros1NodeName);

      messagerHelper.setCommunicationCallbacksEnabled(commsEnabledToStart);
   }

   public StatusLogger getOrCreateStatusLogger()
   {
      if (statusLogger == null)
         statusLogger = new StatusLogger(this::publish);
      return statusLogger;
   }

   // UI Communication Methods:

   @Override
   public <T> void publish(Topic<T> topic, T message)
   {
      messagerHelper.publish(topic, message);
   }

   @Override
   public void publish(Topic<Object> topic)
   {
      messagerHelper.publish(topic);
   }

   @Override
   public ActivationReference<Boolean> subscribeViaActivationReference(Topic<Boolean> topic)
   {
      return messagerHelper.subscribeViaActivationReference(topic);
   }

   @Override
   public <T> void subscribeViaCallback(Topic<T> topic, TopicListener<T> listener)
   {
      messagerHelper.subscribeViaCallback(topic, listener);
   }

   @Override
   public <T> AtomicReference<T> subscribeViaReference(Topic<T> topic, T initialValue)
   {
      return messagerHelper.subscribeViaReference(topic, initialValue);
   }

   @Override
   public Notification subscribeTypelessViaNotification(Topic<Object> topic)
   {
      return messagerHelper.subscribeTypelessViaNotification(topic);
   }

   @Override
   public <T extends K, K> TypedNotification<K> subscribeViaNotification(Topic<T> topic)
   {
      TypedNotification<K> typedNotification = new TypedNotification<>();
      subscribeViaCallback(topic, typedNotification::set);
      return typedNotification;
   }

   public void setCommunicationCallbacksEnabled(boolean enabled)
   {
      super.setCommunicationCallbacksEnabled(enabled);
      messagerHelper.setCommunicationCallbacksEnabled(enabled);
   }

   public MessagerHelper getMessagerHelper()
   {
      return messagerHelper;
   }

   public Messager getMessager()
   {
      return messagerHelper.getMessager();
   }

   public ROS1Helper getROS1Helper()
   {
      return ros1Helper;
   }

   // Behavior Helper Stuff:

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

   public void destroy()
   {
      super.destroy();
      ros1Helper.destroy();
      messagerHelper.disconnect();
   }
}
