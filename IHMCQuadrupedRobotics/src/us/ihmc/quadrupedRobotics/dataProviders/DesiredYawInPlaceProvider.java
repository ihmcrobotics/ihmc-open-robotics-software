package us.ihmc.quadrupedRobotics.dataProviders;


import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.packets.DesiredYawInPlacePacket;
import us.ihmc.quadrupedRobotics.packets.DesiredYawRatePacket;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public class DesiredYawInPlaceProvider implements DoubleProvider, PacketConsumer<DesiredYawInPlacePacket>
{
   private final YoVariableRegistry registry ;
   private final DoubleYoVariable desiredYawInPlaceAmount;
   private final AtomicDouble lastReceivedOffset = new AtomicDouble(0.0);

   public DesiredYawInPlaceProvider(GlobalDataProducer dataProducer, String prefix, YoVariableRegistry parentRegistry)
   {
      if(dataProducer != null)
      {
         dataProducer.attachListener(DesiredYawInPlacePacket.class, this);
      }
      
      String name = YoFrameVariableNameTools.createName(prefix, "desiredYawInPlaceOffset", "");
      registry = new YoVariableRegistry(name + "Provider");
      desiredYawInPlaceAmount = new DoubleYoVariable(name, registry);
      
      parentRegistry.addChild(registry);
      desiredYawInPlaceAmount.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            lastReceivedOffset.set(desiredYawInPlaceAmount.getDoubleValue());
         }
      });
   }
   
   public DesiredYawInPlaceProvider(GlobalDataProducer dataProducer)
   {
      dataProducer.attachListener(DesiredYawInPlacePacket.class, this);
      registry = null;
      desiredYawInPlaceAmount = null;
   }

   @Override public double getValue()
   {
      return lastReceivedOffset.get();
   }

   @Override
   public void receivedPacket(DesiredYawInPlacePacket packet)
   {
      lastReceivedOffset.set(packet.getYawOffset());
   }
   
   public void setToZero()
   {
      lastReceivedOffset.set(0.0);
   }
}