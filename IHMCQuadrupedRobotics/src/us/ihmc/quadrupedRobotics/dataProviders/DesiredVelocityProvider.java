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
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.VectorProvider;

public class DesiredVelocityProvider implements VectorProvider, PacketConsumer<DesiredVelocityPacket>
{
   private final YoVariableRegistry registry = new YoVariableRegistry("desiredVelocityProvider");
   private final YoFrameVector desiredBodyVelocity = new YoFrameVector("desiredBodyVelocity", null, registry);
   private final Vector3d yoVelocityTemporary = new Vector3d();
   private final Vector3d velocity = new Vector3d();
   private final AtomicReference<Vector3d> lastUpdatedVelocity = new AtomicReference<Vector3d>(yoVelocityTemporary);
   
   public DesiredVelocityProvider(GlobalDataProducer dataProducer, YoVariableRegistry parentRegistry)
   {
      dataProducer.attachListener(DesiredVelocityPacket.class, this);
      parentRegistry.addChild(registry);
      desiredBodyVelocity.attachVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            desiredBodyVelocity.get(yoVelocityTemporary);
            lastUpdatedVelocity.set(yoVelocityTemporary);
         }
      });
   }
   
   public DesiredVelocityProvider(GlobalDataProducer dataProducer)
   {
      dataProducer.attachListener(DesiredVelocityPacket.class, this);
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
}
