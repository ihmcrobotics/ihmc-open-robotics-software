package us.ihmc.communication.ros2;

import std_msgs.msg.dds.Empty;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.Timer;
import us.ihmc.tools.thread.Throttler;

import java.util.function.Consumer;

/**
 * Use this class to get the state of a remote {@link ROS2Heartbeat}. We just want
 * to know when that thing is online, active, or something is being requested.
 * This class uses a timer to measure the recency of the last received heartbeat.
 * It also uses an asynchronous thread if a callback is desired, which will at 
 * some rate check and see if the heartbeat is still going and if it changed,
 * callback to the user of this class.
 */
public class ROS2HeartbeatMonitor
{
   /**
    * The expiration needs to be a little longer than the heartbeat to avoid teetering.
    */
   public static final double HEARTBEAT_EXPIRATION = 1.25 * ROS2Heartbeat.HEARTBEAT_PERIOD;
   private final Timer timer = new Timer();
   private final ROS2TypelessInput subscription;

   // To provide callback when aliveness changes
   private volatile boolean running = true;
   private final Throttler throttler = new Throttler();
   private boolean wasAlive = false;
   private Consumer<Boolean> callback = null;
   private final TypedNotification<Boolean> alivenessChangedNotification = new TypedNotification<>();

   public ROS2HeartbeatMonitor(ROS2PublishSubscribeAPI ros2, ROS2Topic<Empty> heartbeatTopic)
   {
      subscription = ros2.subscribeTypeless(heartbeatTopic);
      subscription.addCallback(this::receivedHeartbeat);
      ThreadTools.startAsDaemon(() -> ExceptionTools.handle(this::monitorThread, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE),
                                "HeartbeatMonitor");
   }

   private synchronized void receivedHeartbeat()
   {
      timer.reset();
   }

   /**
    * Used to get the current "alive" status. i.e. "active" or "enabled"
    */
   public synchronized boolean isAlive()
   {
      return timer.isRunning(HEARTBEAT_EXPIRATION);
   }

   /**
    * Start a thread and will callback to the user when the "aliveness" changes.
    * i.e. alive -> not alive or not alive -> alive
    */
   public void setAlivenessChangedCallback(Consumer<Boolean> callback)
   {
      this.callback = callback;
   }

   private void monitorThread()
   {
      while (running)
      {
         throttler.waitAndRun(ROS2Heartbeat.HEARTBEAT_PERIOD);
         boolean isAlive = isAlive();
         if (isAlive != wasAlive)
         {
            wasAlive = isAlive;

            if (callback != null)
               callback.accept(isAlive);
            alivenessChangedNotification.set(isAlive);
         }
      }
   }

   public TypedNotification<Boolean> getAlivenessChangedNotification()
   {
      return alivenessChangedNotification;
   }

   public void destroy()
   {
      running = false;
   }
}
