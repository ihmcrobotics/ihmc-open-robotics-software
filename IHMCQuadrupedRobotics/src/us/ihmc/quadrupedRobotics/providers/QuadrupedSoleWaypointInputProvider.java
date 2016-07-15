package us.ihmc.quadrupedRobotics.providers;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.communication.packets.*;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedSoleWaypointInputProvider
{
   private final AtomicReference<QuadrupedSoleWaypointPacket> quadrupedSoleWaypointPacket;

   public QuadrupedSoleWaypointInputProvider(GlobalDataProducer globalDataProducer, YoVariableRegistry registry)
   {
      quadrupedSoleWaypointPacket = new AtomicReference<>(new QuadrupedSoleWaypointPacket());

      if (globalDataProducer != null)
      {
         globalDataProducer.attachListener(QuadrupedSoleWaypointPacket.class, new PacketConsumer<QuadrupedSoleWaypointPacket>()
         {
            @Override
            public void receivedPacket(QuadrupedSoleWaypointPacket packet)
            {
               quadrupedSoleWaypointPacket.set(packet);
            }
         });
      }
   }

   public QuadrupedSoleWaypointList get()
   {
      return quadrupedSoleWaypointPacket.get().get();
   }
}
