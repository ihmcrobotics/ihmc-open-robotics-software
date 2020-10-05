package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.SingleThreadSizeOneQueueExecutor;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;

import java.util.ArrayList;
import java.util.function.Consumer;

public class ThrottledRobotStateCallback
{
   private final double updatePeriod;
   private final RemoteSyncedRobotModel syncableRobot;
   private final Timer throttleTimer = new Timer();
   private final SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());
   private final ArrayList<Consumer<RemoteSyncedRobotModel>> consumers = new ArrayList<>();
   private final Runnable waitThenAct = this::waitThenAct;

   public ThrottledRobotStateCallback(ROS2Node ros2Node, DRCRobotModel robotModel, double rateHz, Consumer<RemoteSyncedRobotModel> syncedRobotConsumer)
   {
      this(ros2Node, robotModel, rateHz);
      addConsumer(syncedRobotConsumer);
   }

   public ThrottledRobotStateCallback(ROS2Node ros2Node, DRCRobotModel robotModel, double rateHz)
   {
      updatePeriod = UnitConversions.hertzToSeconds(rateHz);
      syncableRobot = new RemoteSyncedRobotModel(robotModel, ros2Node);
      syncableRobot.addRobotConfigurationDataReceivedCallback(() -> executor.queueExecution(waitThenAct));
   }

   private void waitThenAct()
   {
      throttleTimer.sleepUntilExpiration(updatePeriod);
      syncableRobot.update();
      for (Consumer<RemoteSyncedRobotModel> consumer : consumers)
      {
         consumer.accept(syncableRobot);
      }
      throttleTimer.reset();
   }

   public void addConsumer(Consumer<RemoteSyncedRobotModel> syncedRobotConsumer)
   {
      consumers.add(syncedRobotConsumer);
   }
}
