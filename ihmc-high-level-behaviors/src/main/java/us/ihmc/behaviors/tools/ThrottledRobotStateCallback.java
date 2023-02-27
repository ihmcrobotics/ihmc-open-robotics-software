package us.ihmc.behaviors.tools;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.function.Consumer;

public class ThrottledRobotStateCallback
{
   private final double updatePeriod;
   private final ROS2SyncedRobotModel syncedRobot;
   private final Timer throttleTimer = new Timer();
   private final ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ArrayList<Consumer<ROS2SyncedRobotModel>> consumers = new ArrayList<>();
   private final Runnable waitThenAct = this::waitThenAct;

   public ThrottledRobotStateCallback(ROS2NodeInterface ros2Node, DRCRobotModel robotModel, double rateHz, Consumer<ROS2SyncedRobotModel> syncedRobotConsumer)
   {
      this(ros2Node, robotModel, rateHz);
      addConsumer(syncedRobotConsumer);
   }

   public ThrottledRobotStateCallback(ROS2NodeInterface ros2Node, DRCRobotModel robotModel, double rateHz)
   {
      updatePeriod = UnitConversions.hertzToSeconds(rateHz);
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      syncedRobot.addRobotConfigurationDataReceivedCallback(() -> executor.clearQueueAndExecute(waitThenAct));
   }

   private void waitThenAct()
   {
      throttleTimer.sleepUntilExpiration(updatePeriod);
      syncedRobot.update();
      for (Consumer<ROS2SyncedRobotModel> consumer : consumers)
      {
         consumer.accept(syncedRobot);
      }
      throttleTimer.reset();
   }

   public void addConsumer(Consumer<ROS2SyncedRobotModel> syncedRobotConsumer)
   {
      consumers.add(syncedRobotConsumer);
   }

   public void destroy()
   {
      executor.destroy();
   }
}
