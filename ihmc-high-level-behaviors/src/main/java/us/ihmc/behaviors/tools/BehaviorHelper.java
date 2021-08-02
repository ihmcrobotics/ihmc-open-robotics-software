package us.ihmc.behaviors.tools;

import com.google.common.base.CaseFormat;
import controller_msgs.msg.dds.*;
import org.apache.commons.lang.WordUtils;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.BehaviorRegistry;
import us.ihmc.behaviors.tools.interfaces.MessagerPublishSubscribeAPI;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.behaviors.tools.yo.YoBooleanClientHelper;
import us.ihmc.behaviors.tools.yo.YoDoubleClientHelper;
import us.ihmc.behaviors.tools.yo.YoVariableClientPublishSubscribeAPI;
import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.TopicListener;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.ActivationReference;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.utilities.ros.ROS1Helper;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

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
public class BehaviorHelper extends CommunicationHelper implements MessagerPublishSubscribeAPI, YoVariableClientPublishSubscribeAPI
{
   private final ROS1Helper ros1Helper;
   private final MessagerHelper messagerHelper = new MessagerHelper(BehaviorRegistry.getActiveRegistry().getMessagerAPI());
   private final YoVariableClientHelper yoVariableClientHelper;
   private StatusLogger statusLogger;
   private ControllerStatusTracker controllerStatusTracker;

   // TODO: Considerations for ROS 1, Messager, and YoVariableClient with reconnecting
   public BehaviorHelper(String titleCasedBehaviorName, DRCRobotModel robotModel, ROS2NodeInterface ros2Node)
   {
      this(titleCasedBehaviorName, robotModel, ros2Node, true);
   }

   public BehaviorHelper(String titleCasedBehaviorName,
                         DRCRobotModel robotModel,
                         ROS2NodeInterface ros2Node,
                         boolean commsEnabledToStart)
   {
      super(robotModel, ros2Node, commsEnabledToStart);
      String ros1NodeName = CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, titleCasedBehaviorName.replace(" ", ""));
      String yoVariableRegistryName = WordUtils.capitalize(titleCasedBehaviorName).replace(" ", "");
      this.ros1Helper = new ROS1Helper(ros1NodeName);
      yoVariableClientHelper = new YoVariableClientHelper(yoVariableRegistryName);
      messagerHelper.setCommunicationCallbacksEnabled(commsEnabledToStart);
   }

   public StatusLogger getOrCreateStatusLogger()
   {
      if (statusLogger == null)
         statusLogger = new StatusLogger(this::publish);
      return statusLogger;
   }

   public ControllerStatusTracker getOrCreateControllerStatusTracker()
   {
      if (controllerStatusTracker == null)
         controllerStatusTracker = new ControllerStatusTracker(getOrCreateStatusLogger(), getROS2Node(), getRobotModel().getSimpleRobotName());
      return controllerStatusTracker;
   }

   // UI Communication Methods:

   @Override
   public DoubleSupplier subscribeToYoVariableDoubleValue(String variableName)
   {
      return yoVariableClientHelper.subscribeToYoVariableDoubleValue(variableName);
   }

   @Override
   public void publishDoubleValueToYoVariable(String variableName, double value)
   {
      yoVariableClientHelper.publishDoubleValueToYoVariable(variableName, value);
   }

   @Override
   public YoBooleanClientHelper subscribeToYoBoolean(String variableName)
   {
      return yoVariableClientHelper.subscribeToYoBoolean(variableName);
   }

   @Override
   public YoDoubleClientHelper subscribeToYoDouble(String variableName)
   {
      return yoVariableClientHelper.subscribeToYoDouble(variableName);
   }

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

   public <T> AtomicReference<T> subscribeViaReference(Function<String, ROS2Topic<T>> topicFunction, T initialValue)
   {
      AtomicReference<T> reference = new AtomicReference<>(initialValue);
      ros2Helper.subscribeViaCallback(topicFunction, reference::set);
      return reference;
   }

   @Override
   public Notification subscribeTypelessViaNotification(Topic<Object> topic)
   {
      return messagerHelper.subscribeTypelessViaNotification(topic);
   }

   @Override
   public void subscribeViaCallback(Topic<Object> topic, Runnable callback)
   {
      messagerHelper.subscribeViaCallback(topic, callback);
   }

   @Override
   public <T extends K, K> TypedNotification<K> subscribeViaNotification(Topic<T> topic)
   {
      TypedNotification<K> typedNotification = new TypedNotification<>();
      subscribeViaCallback(topic, typedNotification::set);
      return typedNotification;
   }

   public void publishBehaviorControlMode(BehaviorControlModeEnum controlMode)
   {
      BehaviorControlModePacket behaviorControlModePacket = new BehaviorControlModePacket();
      behaviorControlModePacket.setBehaviorControlModeEnumRequest(controlMode.toByte());
      publish(ROS2Tools.getBehaviorControlModeTopic(getRobotModel().getSimpleRobotName()), behaviorControlModePacket);
   }

   public void publishBehaviorType(HumanoidBehaviorType type)
   {
      HumanoidBehaviorTypePacket humanoidBehaviorTypePacket = new HumanoidBehaviorTypePacket();
      humanoidBehaviorTypePacket.setHumanoidBehaviorType(type.toByte());
      publish(ROS2Tools.getBehaviorTypeTopic(getRobotModel().getSimpleRobotName()), humanoidBehaviorTypePacket);
   }

   public void publishToolboxState(Function<String, ROS2Topic<?>> robotNameConsumer, ToolboxState state)
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(state.toByte());
      publish(robotNameConsumer.apply(getRobotModel().getSimpleRobotName()).withTypeName(ToolboxStateMessage.class), toolboxStateMessage);
   }

   public void subscribeToBehaviorStatusViaCallback(Consumer<CurrentBehaviorStatus> callback)
   {
      subscribeViaCallback(ROS2Tools.getBehaviorStatusTopic(getRobotModel().getSimpleRobotName()), behaviorStatusPacket ->
      {
         callback.accept(CurrentBehaviorStatus.fromByte(behaviorStatusPacket.getCurrentBehaviorStatus()));
      });
   }

   public void subscribeToDoorLocationViaCallback(Consumer<DoorLocationPacket> callback)
   {
      subscribeViaCallback(ROS2Tools.getDoorLocationTopic(getRobotModel().getSimpleRobotName()), callback);
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

   public YoVariableClientHelper getYoVariableClientHelper()
   {
      return yoVariableClientHelper;
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
      yoVariableClientHelper.disconnect();
   }
}
