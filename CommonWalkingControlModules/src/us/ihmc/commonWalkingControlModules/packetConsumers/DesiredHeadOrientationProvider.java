package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.sensing.LookAtPacket;
import us.ihmc.communication.packets.walking.HeadOrientationPacket;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class DesiredHeadOrientationProvider implements HeadOrientationProvider
{
   private final AtomicReference<LookAtPacket> lookAtPacket = new AtomicReference<LookAtPacket>();
   private final AtomicReference<HeadOrientationPacket> headOrientationPacket = new AtomicReference<HeadOrientationPacket>();

   private final HeadOrientationPacketConsumer headOrientationPacketConsumer;
   private final LookAtPacketConsumer lookAtPacketConsumer;
   private final ReferenceFrame headOrientationFrame;

   private final ReferenceFrame lookAtFrame = ReferenceFrame.getWorldFrame();
   private final FramePoint pointToTrack = new FramePoint(lookAtFrame);
   private final FrameOrientation headOrientation;
   private final GlobalDataProducer globalDataProducer;

   public DesiredHeadOrientationProvider(ReferenceFrame headOrientationFrame, GlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;
      headOrientationPacketConsumer = new HeadOrientationPacketConsumer();
      lookAtPacketConsumer = new LookAtPacketConsumer();
      this.headOrientationFrame = headOrientationFrame;
      headOrientation = new FrameOrientation(headOrientationFrame);
   }

   public PacketConsumer<HeadOrientationPacket> getHeadOrientationPacketConsumer()
   {
      return headOrientationPacketConsumer;
   }

   public PacketConsumer<LookAtPacket> getLookAtPacketConsumer()
   {
      return lookAtPacketConsumer;
   }

   private class LookAtPacketConsumer implements PacketConsumer<LookAtPacket>
   {
      public void receivedPacket(LookAtPacket object)
      {
         lookAtPacket.set(object);
      }
   }

   private class HeadOrientationPacketConsumer implements PacketConsumer<HeadOrientationPacket>
   {
      public void receivedPacket(HeadOrientationPacket packet)
      {
         if (globalDataProducer != null)
         {
            String errorMessage = PacketValidityChecker.validateHeadOrientationPacket(packet);
            if (errorMessage != null)
            {
               globalDataProducer.notifyInvalidPacketReceived(HeadOrientationPacket.class, errorMessage);
               return;
            }
         }
         headOrientationPacket.set(packet);
      }
   }

   @Override
   public boolean isNewLookAtInformationAvailable()
   {
      LookAtPacket object = lookAtPacket.getAndSet(null);
      if (object != null)
      {
         pointToTrack.setIncludingFrame(lookAtFrame, object.getLookAtPoint());
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public boolean isNewHeadOrientationInformationAvailable()
   {
      HeadOrientationPacket packet = headOrientationPacket.getAndSet(null);
      if (packet != null)
      {
         headOrientation.set(packet.getOrientation());
         return true;
      }
      else
      {
         return false;
      }

   }

   @Override
   public FrameOrientation getDesiredHeadOrientation()
   {
      return headOrientation;
   }

   @Override
   public FramePoint getLookAtPoint()
   {
      return pointToTrack;
   }

   @Override
   public ReferenceFrame getHeadOrientationExpressedInFrame()
   {
      return headOrientationFrame;
   }
}
