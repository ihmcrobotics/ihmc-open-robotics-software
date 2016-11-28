package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packets.TrackablePacket;

public class RequestLidarScanMessage extends TrackablePacket<RequestLidarScanMessage>
{
   public RequestLidarScanMessage()
   {
   }

   @Override
   public boolean epsilonEquals(RequestLidarScanMessage other, double epsilon)
   {
      return true;
   }
}
