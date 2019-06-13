package us.ihmc.atlas.ros;

import java.util.concurrent.atomic.AtomicLong;

import controller_msgs.msg.dds.RobotConfigurationData;
import multisense_ros.StampedPps;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.commons.Conversions;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosTimestampSubscriber;

public class AtlasPPSTimestampOffsetProvider implements DRCROSPPSTimestampOffsetProvider
{
   public static final boolean DEBUG = false;

   private final Object lock = new Object();

   private final String ppsTopic;
   private RosTimestampSubscriber ppsSubscriber;
   private final AtomicLong currentTimeStampOffset = new AtomicLong(0);
   private volatile boolean offsetIsDetermined = false;

   private static AtlasPPSTimestampOffsetProvider instance = null;

   private RosMainNode rosMainNode;
   private long multisensePPSTimestamp = -1;

   private AtlasPPSTimestampOffsetProvider(AtlasSensorInformation sensorInformation)
   {
      ppsTopic = sensorInformation.getPPSRosTopic();

      setupPPSSubscriber();
   }

   private void setupPPSSubscriber()
   {
      ppsSubscriber = new RosTimestampSubscriber()
      {
         @Override
         public void onNewMessage(StampedPps message)
         {
            synchronized (lock)
            {
               multisensePPSTimestamp = message.getHostTime().totalNsecs();
            }
         }
      };
   }

   @Override
   public void attachToRosMainNode(RosMainNode rosMainNode)
   {
      synchronized (lock)
      {
         if (this.rosMainNode == null)
         {
            rosMainNode.attachSubscriber(ppsTopic, ppsSubscriber);
            this.rosMainNode = rosMainNode;
         }
      }
   }

   @Override
   public long getCurrentTimestampOffset()
   {
      return currentTimeStampOffset.get();
   }

   @Override
   public long adjustTimeStampToRobotClock(long timeStamp)
   {
      return timeStamp + currentTimeStampOffset.get();
   }

   @Override
   public long adjustRobotTimeStampToRosClock(long timeStamp)
   {
      return timeStamp - currentTimeStampOffset.get();
   }

   @Override
   public boolean offsetIsDetermined()
   {
      return offsetIsDetermined;
   }

   public synchronized static DRCROSPPSTimestampOffsetProvider getInstance(AtlasSensorInformation sensorInformation)
   {
      if (instance == null)
         instance = new AtlasPPSTimestampOffsetProvider(sensorInformation);
      return instance;
   }

   @Override
   public void receivedPacket(RobotConfigurationData packet)
   {
      synchronized (lock)
      {
         if (rosMainNode != null)// && rosMainNode.isStarted()
         {
            long lastPPSTimestampFromRobot = packet.getSyncTimestamp();

            long ppsFromRobotAge = packet.getMonotonicTime() - lastPPSTimestampFromRobot;
            long ppsFromROSAge = rosMainNode.getCurrentTime().totalNsecs() - multisensePPSTimestamp;

            // Check if timestamps are approximately the same age. If not, we discard this measurement and wait for the next one.
            if (Math.abs(ppsFromRobotAge - ppsFromROSAge) > Conversions.millisecondsToNanoseconds(100))
            {
               if (DEBUG)
               {
                  System.out.println("[AtlasPPSTimestampOffsetProvider] Last received pps (robot age: " + Conversions.nanosecondsToSeconds(ppsFromRobotAge)
                        + ", ros age:" + Conversions.nanosecondsToSeconds(ppsFromROSAge) + ") are not of the same signal. Ignoring this measurement. Current offset is " + currentTimeStampOffset.get());
               }
               return;
            }
            else
            {
               // Timestamps are probably from the same pulse, updating offset
               long newOffset = lastPPSTimestampFromRobot - multisensePPSTimestamp;
               long lastTimestampOffset = currentTimeStampOffset.getAndSet(newOffset);

               if (offsetIsDetermined && Math.abs(newOffset - lastTimestampOffset) > Conversions.millisecondsToNanoseconds(1))
               {
                  System.err.println("[AtlasPPSTimestampOffsetProvider] Unstable PPS offset. New offset: " + newOffset + ", previous offset: "
                        + lastTimestampOffset);
               }

               offsetIsDetermined = true;
            }

         }
      }
   }

}