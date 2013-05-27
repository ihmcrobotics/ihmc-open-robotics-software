package us.ihmc.commonWalkingControlModules.controlModules.head;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

/**
 * User: Matt
 * Date: 1/28/13
 */
public class DesiredHeadOrientationProvider
{
   private HeadOrientationControlModule headOrientationControlModule;

   
   private final  ObjectConsumer<HeadOrientationPacket> headOrientationPacketConsumer;
   private final ObjectConsumer<LookAtPacket> lookAtPacketConsumer;
   private final ReferenceFrame lookAtFrame = ReferenceFrame.getWorldFrame();
   private ReferenceFrame headOrientationFrame;
   
   
   private double desiredJointForExtendedNeckPitchRangeAngle = 0.0;

   public DesiredHeadOrientationProvider()
   {
      headOrientationPacketConsumer = new HeadOrientationPacketConsumer();
      lookAtPacketConsumer = new LookAtPacketConsumer();
   }

   public ObjectConsumer<HeadOrientationPacket> getHeadOrientationPacketConsumer()
   {
      return headOrientationPacketConsumer;
   }
   

   public ObjectConsumer<LookAtPacket> getLookAtPacketConsumer()
   {
      return lookAtPacketConsumer;
   }

   public void setHeadOrientationControlModule(HeadOrientationControlModule headOrientationControlModule)
   {
      this.headOrientationControlModule = headOrientationControlModule;
      
      this.headOrientationFrame = headOrientationControlModule.getHeadOrientationFrame();
   }
   
   private class LookAtPacketConsumer implements ObjectConsumer<LookAtPacket>
   {

      public void consumeObject(LookAtPacket object)
      {
         if (headOrientationControlModule != null)
         {
            FramePoint pointToTrack = new FramePoint(lookAtFrame, object.getLookAtPoint());
            headOrientationControlModule.setPointToTrack(pointToTrack);
            
            desiredJointForExtendedNeckPitchRangeAngle = 0.0;
         }         
      }
      
   }

   private class HeadOrientationPacketConsumer implements ObjectConsumer<HeadOrientationPacket>
   {
      public void consumeObject(HeadOrientationPacket packet)
      {
         if (headOrientationControlModule != null)
         {
            FrameOrientation frameOrientation = new FrameOrientation(headOrientationFrame, packet.getQuaternion());
            headOrientationControlModule.setOrientationToTrack(frameOrientation);
 
            desiredJointForExtendedNeckPitchRangeAngle = packet.getDesiredJointForExtendedNeckPitchRangeAngle();
         }
      }
   }

   public double getDesiredExtendedNeckPitchJointAngle()
   {
      return desiredJointForExtendedNeckPitchRangeAngle;
   }

}
