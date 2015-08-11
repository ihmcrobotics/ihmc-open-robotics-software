package us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.realtime.concurrent.Builder;
import us.ihmc.realtime.concurrent.ConcurrentCopier;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandJointAngleCommunicator
{
   private final int WORKER_SLEEP_TIME_MILLIS = 500;

   private final PacketCommunicator packetCommunicator;
   private final GlobalDataProducer dataProducer;

   private final ConcurrentCopier<HandJointAnglePacket> packetCopier;
   private double[][] fingers = new double[3][];
   private final AtomicBoolean connected = new AtomicBoolean();
   private final AtomicBoolean calibrated = new AtomicBoolean();
   private final RobotSide side;

   public HandJointAngleCommunicator(RobotSide side, PacketCommunicator packetCommunicator)
   {
      this(side, packetCommunicator, null);
   }

   public HandJointAngleCommunicator(RobotSide side, GlobalDataProducer dataProducer)
   {
      this(side, null, dataProducer);
   }

   private HandJointAngleCommunicator(RobotSide side, PacketCommunicator packetCommunicator, GlobalDataProducer dataProducer)
   {
      this.side = side;
      this.packetCommunicator = packetCommunicator;
      this.dataProducer = dataProducer;
      packetCopier = new ConcurrentCopier<HandJointAnglePacket>(HandJointAngleCommunicator.builder);
      startWriterThread();
   }

   private void startWriterThread()
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
               if (packetCommunicator != null)
               {
                  packetCommunicator.send(copyForReading);
               }
               else
               {
                  dataProducer.send(copyForReading);
               }
            }
         }
      }, 0, WORKER_SLEEP_TIME_MILLIS, TimeUnit.MILLISECONDS);
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
      HandJointAnglePacket packet = packetCopier.getCopyForWriting();
      if (packet == null)
      {
         return;
      }
      packet.setAll(side, connected.get(), calibrated.get(), fingers[0], fingers[1], fingers[2]);
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
}