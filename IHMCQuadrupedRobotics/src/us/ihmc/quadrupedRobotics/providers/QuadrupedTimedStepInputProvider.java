package us.ihmc.quadrupedRobotics.providers;

import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedTimedStepPacket;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedTimedStepInputProvider
{
   private final AtomicReference<QuadrupedTimedStepPacket> timedStepPacket;

   public QuadrupedTimedStepInputProvider(GlobalDataProducer globalDataProducer, YoVariableRegistry registry)
   {
      timedStepPacket = new AtomicReference<>(new QuadrupedTimedStepPacket());

      if (globalDataProducer != null)
      {
         globalDataProducer.attachListener(QuadrupedTimedStepPacket.class, new PacketConsumer<QuadrupedTimedStepPacket>()
         {
            @Override
            public void receivedPacket(QuadrupedTimedStepPacket packet)
            {
               timedStepPacket.set(packet);
            }
         });
      }
   }

   public ArrayList<QuadrupedTimedStep> get()
   {
      return timedStepPacket.get().get();
   }
}
