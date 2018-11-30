package us.ihmc.humanoidRobotics.communication.subscribers;

import java.util.concurrent.atomic.AtomicBoolean;

import controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket;
import us.ihmc.communication.net.PacketConsumer;

public class RequestWristForceSensorCalibrationSubscriber implements PacketConsumer<RequestWristForceSensorCalibrationPacket>
{
   private final AtomicBoolean hasReceivedNewCalibrationRequest = new AtomicBoolean(false);

   public RequestWristForceSensorCalibrationSubscriber()
   {
   }

   public boolean checkForNewCalibrationRequest()
   {
      return hasReceivedNewCalibrationRequest.getAndSet(false);
   }

   @Override
   public void receivedPacket(RequestWristForceSensorCalibrationPacket object)
   {
      if (object != null)
         hasReceivedNewCalibrationRequest.set(true);
   }
}
