package us.ihmc.quadrupedRobotics.dataProviders;


import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.packets.DesiredYawRatePacket;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public class DesiredYawRateProvider implements DoubleProvider, PacketConsumer<DesiredYawRatePacket>
{
   private final YoVariableRegistry registry ;
   private final DoubleYoVariable desiredYawRate;
   private final AtomicDouble lastReceivedYawRate = new AtomicDouble(0.0);

   public DesiredYawRateProvider(GlobalDataProducer dataProducer, String prefix, YoVariableRegistry parentRegistry)
   {
      if(dataProducer != null)
      {
         dataProducer.attachListener(DesiredYawRatePacket.class, this);
      }
      
      String name = YoFrameVariableNameTools.createName(prefix, "desiredYawRate", "");
      registry = new YoVariableRegistry(name + "Provider");
      desiredYawRate = new DoubleYoVariable(name, registry);
      
      parentRegistry.addChild(registry);
      desiredYawRate.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            lastReceivedYawRate.set(desiredYawRate.getDoubleValue());
         }
      });
   }
   
   public DesiredYawRateProvider(GlobalDataProducer dataProducer)
   {
      dataProducer.attachListener(DesiredYawRatePacket.class, this);
      registry = null;
      desiredYawRate = null;
   }

   @Override public double getValue()
   {
      return lastReceivedYawRate.get();
   }

   @Override
   public void receivedPacket(DesiredYawRatePacket packet)
   {
      lastReceivedYawRate.set(packet.getYawRate());
   }
}