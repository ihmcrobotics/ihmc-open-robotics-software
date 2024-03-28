package us.ihmc.behaviors.tools;

import org.apache.commons.lang.WordUtils;
import perception_msgs.msg.dds.DoorLocationPacket;
import toolbox_msgs.msg.dds.BehaviorControlModePacket;
import toolbox_msgs.msg.dds.BehaviorStatusPacket;
import toolbox_msgs.msg.dds.HumanoidBehaviorTypePacket;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.BehaviorModule;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.behaviors.tools.yo.YoBooleanClientHelper;
import us.ihmc.behaviors.tools.yo.YoDoubleClientHelper;
import us.ihmc.behaviors.tools.yo.YoVariableClientPublishSubscribeAPI;
import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.communication.DeprecatedAPIs;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.PausablePeriodicThread;

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
 */
public class BehaviorHelper extends CommunicationHelper implements YoVariableClientPublishSubscribeAPI
{
   public static final ROS2Topic<?> BEHAVIOR_MODULE_INPUT = DeprecatedAPIs.BEHAVIOR_MODULE.withInput();
   public static final ROS2Topic<?> BEHAVIOR_MODULE_OUTPUT = DeprecatedAPIs.BEHAVIOR_MODULE.withOutput();
   public static final ROS2Topic<BehaviorControlModePacket> BEHAVIOR_CONTROL_MODE = BEHAVIOR_MODULE_INPUT.withTypeName(BehaviorControlModePacket.class);
   public static final ROS2Topic<HumanoidBehaviorTypePacket> BEHAVIOR_TYPE = BEHAVIOR_MODULE_INPUT.withTypeName(HumanoidBehaviorTypePacket.class);
   public static final ROS2Topic<BehaviorStatusPacket> BEHAVIOR_STATUS = BEHAVIOR_MODULE_OUTPUT.withTypeName(BehaviorStatusPacket.class);

   private final YoVariableClientHelper yoVariableClientHelper;
   private StatusLogger statusLogger;
   private ControllerStatusTracker controllerStatusTracker;

   // TODO: Considerations for YoVariableClient with reconnecting
   public BehaviorHelper(String titleCasedBehaviorName, DRCRobotModel robotModel, ROS2NodeInterface ros2Node)
   {
      super(robotModel, ros2Node);
      String yoVariableRegistryName = WordUtils.capitalize(titleCasedBehaviorName).replace(" ", "");
      yoVariableClientHelper = new YoVariableClientHelper(yoVariableRegistryName);
   }

   public StatusLogger getOrCreateStatusLogger()
   {
      if (statusLogger == null)
      {
         statusLogger = new StatusLogger((topic, message) -> publish(BehaviorModule.API.STATUS_LOG, message));
      }
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

   public <T> AtomicReference<T> subscribeViaReference(Function<String, ROS2Topic<T>> topicFunction, T initialValue)
   {
      AtomicReference<T> reference = new AtomicReference<>(initialValue);
      ros2Helper.subscribeViaCallback(topicFunction, reference::set);
      return reference;
   }

   public void publishBehaviorControlMode(BehaviorControlModeEnum controlMode)
   {
      BehaviorControlModePacket behaviorControlModePacket = new BehaviorControlModePacket();
      behaviorControlModePacket.setBehaviorControlModeEnumRequest(controlMode.toByte());
      String robotName = getRobotModel().getSimpleRobotName();
      publish(BEHAVIOR_CONTROL_MODE.withRobot(robotName), behaviorControlModePacket);
   }

   public void publishBehaviorType(HumanoidBehaviorType type)
   {
      HumanoidBehaviorTypePacket humanoidBehaviorTypePacket = new HumanoidBehaviorTypePacket();
      humanoidBehaviorTypePacket.setHumanoidBehaviorType(type.toByte());
      String robotName = getRobotModel().getSimpleRobotName();
      publish(BEHAVIOR_TYPE.withRobot(robotName), humanoidBehaviorTypePacket);
   }

   public void publishToolboxState(Function<String, ROS2Topic<?>> robotNameConsumer, ToolboxState state)
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(state.toByte());
      publish(robotNameConsumer.apply(getRobotModel().getSimpleRobotName()).withTypeName(ToolboxStateMessage.class), toolboxStateMessage);
   }

   public void subscribeToBehaviorStatusViaCallback(Consumer<CurrentBehaviorStatus> callback)
   {
      String robotName = getRobotModel().getSimpleRobotName();
      subscribeViaCallback(BEHAVIOR_STATUS.withRobot(robotName), behaviorStatusPacket ->
      {
         callback.accept(CurrentBehaviorStatus.fromByte(behaviorStatusPacket.getCurrentBehaviorStatus()));
      });
   }

   public void subscribeToDoorLocationViaCallback(Consumer<DoorLocationPacket> callback)
   {
      subscribeViaCallback(PerceptionAPI.getDoorLocationTopic(getRobotModel().getSimpleRobotName()), callback);
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
      yoVariableClientHelper.disconnect();
   }
}
