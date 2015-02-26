package us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.simulationconstructionset.robotController.RawOutputWriter;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class HandJointAngleCommunicator implements RawOutputWriter
{
   private final int WORKER_SLEEP_TIME_MILLIS = 250;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final PacketCommunicator packetCommunicator;
   private final ConcurrentCopier<HandJointAnglePacket> packetCopier;
   private double[][] fingers = new double[3][];
   private final AtomicBoolean connected = new AtomicBoolean();
   private final RobotSide side;

   public HandJointAngleCommunicator(RobotSide side, PacketCommunicator networkProcessorCommunicator)
   {
      this.side = side;
      this.packetCommunicator = networkProcessorCommunicator;
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
            if (packetCommunicator != null)
               packetCommunicator.send(packetCopier.getCopyForReading());
         }
      }, 0, WORKER_SLEEP_TIME_MILLIS, TimeUnit.MILLISECONDS);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   public void updateHandAngles(HandSensorData sensorDataFromHand)
   {
	   fingers = sensorDataFromHand.getFingerJointAngles();
	   
	   connected.set(true);
   }

   @Override
   public void write()
   {
      HandJointAnglePacket packet = packetCopier.getCopyForWriting();
      if (packet == null)
      {
         return;
      }
      packet.setAll(side, connected.get(), fingers[0], fingers[1], fingers[2]);
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