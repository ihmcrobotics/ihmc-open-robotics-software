package us.ihmc.aware.params;

import us.ihmc.aware.packets.SetDoubleParameterPacket;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;

public class ParameterPacketListener
{
   // TODO: Make this a packet communicator, not a global data producer?
   public ParameterPacketListener(GlobalDataProducer communicator)
   {
      communicator.attachListener(SetDoubleParameterPacket.class, new PacketConsumer<SetDoubleParameterPacket>()
      {
         @Override
         public void receivedPacket(SetDoubleParameterPacket packet)
         {
            Parameter parameter = ParameterRegistry.getInstance().getParameter(packet.getParameterName());

            if (parameter == null)
            {
               // TODO: Error -- tried to set nonexistent parameter
               System.err.println("Packet tried to set nonexistent parameter: " + packet.getParameterName());
            }
            else if (!(parameter instanceof DoubleParameter))
            {
               // TODO: Error -- tried to set wrong parameter type
               System.err.println("Packet tried setting parameter of wrong type: " + packet.getParameterName());
            }
            else
            {
               ((DoubleParameter) parameter).set(packet.getParameterValue());
               System.out.println("Setting " + parameter.getPath() + " to " + packet.getParameterValue());
            }
         }
      });
   }
}
