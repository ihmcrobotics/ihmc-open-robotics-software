package us.ihmc.quadrupedRobotics.dataProviders;

import us.ihmc.communication.packetCommunicator.ConcurrentPacketQueue;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.packets.DesiredVelocityPacket;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.VectorProvider;

import javax.vecmath.Vector3d;

public class DesiredVelocityProvider implements VectorProvider
{
   private final ConcurrentPacketQueue<DesiredVelocityPacket> packetQueue = new ConcurrentPacketQueue<>();
   private FrameVector lastReceivedVelocity = new FrameVector(ReferenceFrame.getWorldFrame(), new Vector3d());

   public DesiredVelocityProvider(GlobalDataProducer dataProducer)
   {
      dataProducer.attachListener(DesiredVelocityPacket.class, packetQueue);
   }

   @Override public void get(FrameVector frameVectorToPack)
   {
      if(packetQueue.isNewPacketAvailable())
         lastReceivedVelocity.set(packetQueue.getPacket().getVelocity());

      frameVectorToPack.setIncludingFrame(lastReceivedVelocity);
   }
}
