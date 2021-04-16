package us.ihmc.behaviors.tools;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.TopicListener;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.tools.thread.ActivationReference;
import us.ihmc.utilities.ros.RosNodeInterface;

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
public class BehaviorHelper extends CommunicationHelper
{
   private ManagedMessager managedMessager;
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
      super(robotModel, ros1Node, ros2Node, commsEnabledToStart);
      if (messager != null)
      {
         managedMessager = new ManagedMessager(messager);
      }

      setCommunicationCallbacksEnabled(commsEnabledToStart);
   }

   public StatusLogger getOrCreateStatusLogger()
   {
      if (statusLogger == null)
         statusLogger = new StatusLogger(this::publish);
      return statusLogger;
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
      subscribeViaCallback(topic, typedNotification::set);
      return typedNotification;
   }

   public void setCommunicationCallbacksEnabled(boolean enabled)
   {
      super.setCommunicationCallbacksEnabled(enabled);
      if (managedMessager != null)
      {
         managedMessager.setEnabled(enabled);
      }
   }

   public void setNewMessager(Messager messager)
   {
      boolean enabled = managedROS2Node.isEnabled();
      managedMessager = new ManagedMessager(messager);
      managedMessager.setEnabled(enabled);
   }

   public Messager getMessager()
   {
      return managedMessager;
   }
}
