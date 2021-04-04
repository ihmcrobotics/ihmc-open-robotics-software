package us.ihmc.behaviors.tools;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
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
   private final RemoteSyncedRobotModel syncableRobot;
   private final Timer throttleTimer = new Timer();
   private final ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ArrayList<Consumer<RemoteSyncedRobotModel>> consumers = new ArrayList<>();
   private final Runnable waitThenAct = this::waitThenAct;

   public ThrottledRobotStateCallback(ROS2NodeInterface ros2Node, DRCRobotModel robotModel, double rateHz, Consumer<RemoteSyncedRobotModel> syncedRobotConsumer)
   {
      this(ros2Node, robotModel, rateHz);
      addConsumer(syncedRobotConsumer);
   }

   public ThrottledRobotStateCallback(ROS2NodeInterface ros2Node, DRCRobotModel robotModel, double rateHz)
   {
      updatePeriod = UnitConversions.hertzToSeconds(rateHz);
      syncableRobot = new RemoteSyncedRobotModel(robotModel, ros2Node);
      syncableRobot.addRobotConfigurationDataReceivedCallback(() -> executor.clearQueueAndExecute(waitThenAct));
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

   public void destroy()
   {
      executor.destroy();
   }
}
