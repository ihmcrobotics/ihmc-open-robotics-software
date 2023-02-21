package us.ihmc.communication.ros2;

import std_msgs.msg.dds.Empty;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.Timer;
import us.ihmc.tools.thread.Throttler;

import java.util.function.Consumer;

/**
 * Use this class to get the state of a remote heartbeat. We just want
 * to know when that thing is online, active, or something is being requested.
 */
public class ROS2HeartbeatMonitor
{
   /**
    * The expiration needs to be a little longer than the heartbeat to avoid teetering.
    */
   public static final double HEARTBEAT_EXPIRATION = 1.25 * ROS2Heartbeat.HEARTBEAT_PERIOD;
   private final Timer timer = new Timer();

   // To provide callback when aliveness changes
   private boolean monitorThreadStarted = false;
   private final Throttler throttler = new Throttler();
   private boolean wasAlive = false;
   private Consumer<Boolean> callback = null;

   public ROS2HeartbeatMonitor(ROS2PublishSubscribeAPI ros2, ROS2Topic<Empty> heartbeatTopic)
   {
      ros2.subscribeViaCallback(heartbeatTopic, this::receivedHeartbeat);
   }

   private synchronized void receivedHeartbeat()
   {
      timer.reset();
   }

   public synchronized boolean isAlive()
   {
      return timer.isRunning(HEARTBEAT_EXPIRATION);
   }

   public void setAlivenessChangedCallback(Consumer<Boolean> callback)
   {
      if (!monitorThreadStarted)
      {
         monitorThreadStarted = true;
         ThreadTools.startAsDaemon(() -> ExceptionTools.handle(this::monitorThread, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE),
                                   "HeartbeatMonitor");
      }

      this.callback = callback;
   }

   private void monitorThread()
   {
      while (true)
      {
         throttler.waitAndRun(ROS2Heartbeat.HEARTBEAT_PERIOD);
         boolean isAlive = isAlive();
         if (isAlive != wasAlive)
         {
            wasAlive = isAlive;

            if (callback != null)
               callback.accept(isAlive);
         }
      }
   }
}
