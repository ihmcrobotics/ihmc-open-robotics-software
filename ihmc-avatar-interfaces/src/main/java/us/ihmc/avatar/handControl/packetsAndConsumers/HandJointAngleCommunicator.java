package us.ihmc.avatar.handControl.packetsAndConsumers;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import controller_msgs.msg.dds.HandJointAnglePacket;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandJointAngleCommunicator
{
   private final int WORKER_SLEEP_TIME_MILLIS = 500;

   private final ConcurrentCopier<HandJointAnglePacket> packetCopier;
   private double[] fingers;
   private final AtomicBoolean connected = new AtomicBoolean();
   private final AtomicBoolean calibrated = new AtomicBoolean();
   private final RobotSide side;

   private final ScheduledExecutorService executor;

   private final IHMCRealtimeROS2Publisher<HandJointAnglePacket> publisher;

   public HandJointAngleCommunicator(RobotSide side, IHMCRealtimeROS2Publisher<HandJointAnglePacket> publisher)
   {
      this.side = side;
      this.publisher = publisher;
      packetCopier = new ConcurrentCopier<HandJointAnglePacket>(HandJointAngleCommunicator.builder);
      executor = startWriterThread();
   }

   private ScheduledExecutorService startWriterThread()
   {
      ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
      executor.scheduleAtFixedRate(new Runnable()
      {
         @Override
         public void run()
         {
            HandJointAnglePacket copyForReading = packetCopier.getCopyForReading();
            if (copyForReading != null)
            {
               if (publisher != null)
               {
                  publisher.publish(copyForReading);
               }
            }
         }
      }, 0, WORKER_SLEEP_TIME_MILLIS, TimeUnit.MILLISECONDS);

      return executor;
   }

   public String getName()
   {
      return getClass().getSimpleName();
   }

   public String getDescription()
   {
      return getName();
   }

   public void updateHandAngles(HandSensorData sensorDataFromHand)
   {
      fingers = sensorDataFromHand.getFingerJointAngles(side);
      calibrated.set(sensorDataFromHand.isCalibrated());
      connected.set(sensorDataFromHand.isConnected());
   }

   public void write()
   {
      if (fingers == null)
      {
         return;
      }

      HandJointAnglePacket packet = packetCopier.getCopyForWriting();
      if (packet == null)
      {
         return;
      }
      packet.setRobotSide(side.toByte());
      packet.setConnected(connected.get());
      packet.setCalibrated(calibrated.get());
      packet.getJointAngles().reset();
      packet.getJointAngles().add(fingers);
      packetCopier.commit();
   }

   public static final Builder<HandJointAnglePacket> builder = new Builder<HandJointAnglePacket>()
   {
      @Override
      public HandJointAnglePacket newInstance()
      {
         return new HandJointAnglePacket();
      }
   };

   public void setHandDisconnected()
   {
      connected.set(true);
   }

   public void cleanup()
   {
      executor.shutdown();
   }
}