package us.ihmc.quadrupedRobotics.dataProviders;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Vector3d;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.packets.DesiredVelocityPacket;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.trajectories.providers.VectorProvider;

public class DesiredVelocityProvider implements VectorProvider, PacketConsumer<DesiredVelocityPacket>
{
   private final YoVariableRegistry registry;
   private final YoFrameVector desiredVelocity;
   private final Vector3d yoVelocityTemporary = new Vector3d();
   private final Vector3d velocity = new Vector3d();
   private final Vector3d zeroVelocity = new Vector3d();
   private final AtomicReference<Vector3d> lastUpdatedVelocity = new AtomicReference<Vector3d>(yoVelocityTemporary);
   
   public DesiredVelocityProvider(GlobalDataProducer dataProducer, String prefix, YoVariableRegistry parentRegistry)
   {
      if(dataProducer != null)
      {
         dataProducer.attachListener(DesiredVelocityPacket.class, this);
      }
      
      String name = YoFrameVariableNameTools.createName(prefix, "desiredVelocity", "");
      registry = new YoVariableRegistry(name + "Provider");
      desiredVelocity = new YoFrameVector(name, null, registry);
      desiredVelocity.attachVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            desiredVelocity.get(yoVelocityTemporary);
            lastUpdatedVelocity.set(yoVelocityTemporary);
         }
      });
      parentRegistry.addChild(registry);
   }
   
   public DesiredVelocityProvider(GlobalDataProducer dataProducer)
   {
      dataProducer.attachListener(DesiredVelocityPacket.class, this);
      registry = null;
      desiredVelocity = null;
   }

   @Override 
   public void get(FrameVector frameVectorToPack)
   {
      velocity.set(lastUpdatedVelocity.get());      
      frameVectorToPack.set(velocity);
   }

   @Override
   public void receivedPacket(DesiredVelocityPacket packet)
   {
      lastUpdatedVelocity.set(packet.getVelocity());
   }
   
   public void setToZero()
   {
      lastUpdatedVelocity.set(zeroVelocity);
   }
}
