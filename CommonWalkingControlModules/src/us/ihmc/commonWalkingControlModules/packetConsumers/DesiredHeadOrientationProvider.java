package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.commonWalkingControlModules.packets.HeadOrientationPacket;
import us.ihmc.commonWalkingControlModules.packets.LookAtPacket;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredHeadOrientationProvider
{
   private final Object synchronizationObject = new Object();

   private final HeadOrientationPacketConsumer headOrientationPacketConsumer;
   private final LookAtPacketConsumer lookAtPacketConsumer;
   private final ReferenceFrame headOrientationFrame;

   private boolean isNewLookAtInformationAvailable;
   private boolean isNewHeadOrientationInformationAvailable;
   private double desiredJointForExtendedNeckPitchRangeAngle;

   public DesiredHeadOrientationProvider(ReferenceFrame headOrientationFrame)
   {
      headOrientationPacketConsumer = new HeadOrientationPacketConsumer(headOrientationFrame);
      lookAtPacketConsumer = new LookAtPacketConsumer();
      this.headOrientationFrame = headOrientationFrame;
   }

   public ObjectConsumer<HeadOrientationPacket> getHeadOrientationPacketConsumer()
   {
      return headOrientationPacketConsumer;
   }

   public ObjectConsumer<LookAtPacket> getLookAtPacketConsumer()
   {
      return lookAtPacketConsumer;
   }

   private class LookAtPacketConsumer implements ObjectConsumer<LookAtPacket>
   {
      private final ReferenceFrame lookAtFrame = ReferenceFrame.getWorldFrame();

      private final FramePoint pointToTrack = new FramePoint(lookAtFrame);

      public void consumeObject(LookAtPacket object)
      {
         synchronized (synchronizationObject)
         {
            pointToTrack.set(lookAtFrame, object.getLookAtPoint());
            desiredJointForExtendedNeckPitchRangeAngle = 0.0;
            isNewLookAtInformationAvailable = true;
            isNewHeadOrientationInformationAvailable = false;
         }
      }
   }


   private class HeadOrientationPacketConsumer implements ObjectConsumer<HeadOrientationPacket>
   {
      private final FrameOrientation headOrientation;

      public HeadOrientationPacketConsumer(ReferenceFrame headOrientationExpressedInFrame)
      {
         headOrientation = new FrameOrientation(headOrientationExpressedInFrame);
      }

      public void consumeObject(HeadOrientationPacket packet)
      {
         synchronized (synchronizationObject)
         {
            headOrientation.set(packet.getQuaternion());
            desiredJointForExtendedNeckPitchRangeAngle = packet.getDesiredJointForExtendedNeckPitchRangeAngle();

            isNewLookAtInformationAvailable = false;
            isNewHeadOrientationInformationAvailable = true;
         }
      }
   }


   public boolean isNewLookAtInformationAvailable()
   {
      synchronized (synchronizationObject)
      {
         return isNewLookAtInformationAvailable;
      }
   }

   public boolean isNewHeadOrientationInformationAvailable()
   {
      synchronized (synchronizationObject)
      {
         return isNewHeadOrientationInformationAvailable;
      }
   }

   public FrameOrientation getDesiredHeadOrientation()
   {
      synchronized (synchronizationObject)
      {
         isNewHeadOrientationInformationAvailable = false;

         return headOrientationPacketConsumer.headOrientation;
      }
   }

   public FramePoint getLookAtPoint()
   {
      synchronized (synchronizationObject)
      {
         isNewLookAtInformationAvailable = false;

         return lookAtPacketConsumer.pointToTrack;
      }
   }

   public ReferenceFrame getHeadOrientationExpressedInFrame()
   {
      return headOrientationFrame;
   }

   public double getDesiredExtendedNeckPitchJointAngle()
   {
      synchronized (synchronizationObject)
      {
         return desiredJointForExtendedNeckPitchRangeAngle;
      }
   }

}
