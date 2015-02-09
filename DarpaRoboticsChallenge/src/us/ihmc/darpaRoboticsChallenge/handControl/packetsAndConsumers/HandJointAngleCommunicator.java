package us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers;

import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.simulationconstructionset.robotController.RawOutputWriter;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

// fills a ring buffer with pose and joint data and in a worker thread passes it to the appropriate consumer 
public class HandJointAngleCommunicator implements RawOutputWriter
{
   private final int WORKER_SLEEP_TIME_MILLIS = 250;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final PacketCommunicator networkProcessorCommunicator;
   private final ConcurrentRingBuffer<HandJointAnglePacket> packetRingBuffer;
   private double[][] fingers = new double[3][];
   private final AtomicBoolean connected = new AtomicBoolean();
   private final RobotSide side;
   private HandJointAnglePacket currentPacket;

   public HandJointAngleCommunicator(RobotSide side, PacketCommunicator networkProcessorCommunicator)
   {
      this.side = side;
      this.networkProcessorCommunicator = networkProcessorCommunicator;
      packetRingBuffer = new ConcurrentRingBuffer<HandJointAnglePacket>(HandJointAngleCommunicator.builder, 8);
//      startWriterThread();
   }

   // this thread reads from the stateRingBuffer and pushes the data out to the objectConsumer
//   private void startWriterThread()
//   {
//      AsyncContinuousExecutor.executeContinuously(new Runnable()
//      {
//         @Override
//         public void run()
//         {
//            if (packetRingBuffer.poll())
//            {
//               while ((currentPacket = packetRingBuffer.read()) != null)
//               {
//                  if (networkProcessorCommunicator == null)
//                  {
//                     System.out.println("Net Proc Comm");
//                  }
//                  networkProcessorCommunicator.send(currentPacket);
//               }
//               packetRingBuffer.flush();
//            }
//         }
//      }, WORKER_SLEEP_TIME_MILLIS, "Hand Joint Angle Communicator");
//   }

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

   // puts the state data into the ring buffer for the output thread
   @Override
   public void write()
   {
//      HandJointAnglePacket packet = packetRingBuffer.next();
//      if (packet == null)
//      {
//         return;
//      }
//      packet.setAll(side, connected.get(), fingers[0], fingers[1], fingers[2]);
//      packetRingBuffer.commit();
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