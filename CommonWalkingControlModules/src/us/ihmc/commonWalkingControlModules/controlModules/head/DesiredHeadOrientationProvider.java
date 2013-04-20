package us.ihmc.commonWalkingControlModules.controlModules.head;

import java.util.HashMap;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;
import us.ihmc.utilities.screwTheory.RigidBody;

/**
 * User: Matt
 * Date: 1/28/13
 */
public class DesiredHeadOrientationProvider
{
   private HeadOrientationControlModule headOrientationControlModule;

   private ObjectConsumer<HeadOrientationPacket> headOrientationPacketConsumer;
   private final HashMap<String, ReferenceFrame> availableHeadControlFrames = new HashMap<String, ReferenceFrame>();
   private final HashMap<String, RigidBody> availableBases = new HashMap<String, RigidBody>();
   
   private double desiredJointForExtendedNeckPitchRangeAngle = 0.0;

   public DesiredHeadOrientationProvider()
   {
      headOrientationPacketConsumer = new HeadOrientationPacketConsumer();
   }

   public ObjectConsumer<HeadOrientationPacket> getHeadOrientationPacketConsumer()
   {
      return headOrientationPacketConsumer;
   }

   public void setHeadOrientationControlModule(HeadOrientationControlModule headOrientationControlModule)
   {
      this.headOrientationControlModule = headOrientationControlModule;
      
      for (ReferenceFrame frame : headOrientationControlModule.getAvailableHeadControlFrames())
      {
         this.availableHeadControlFrames.put(frame.getName(), frame);
      }

      for (RigidBody base : headOrientationControlModule.getAvailableBases())
      {
         this.availableBases.put(base.getName(), base);
      }
   }

   private class HeadOrientationPacketConsumer implements ObjectConsumer<HeadOrientationPacket>
   {
      public void consumeObject(HeadOrientationPacket packet)
      {
         if (headOrientationControlModule != null)
         {
            ReferenceFrame frame = availableHeadControlFrames.get(packet.getFrameName());
            if (frame == null)
               throw new RuntimeException("Frame with name " + packet.getFrameName() + " is not an available head control frame");

            RigidBody base = availableBases.get(packet.getBaseName());
            if (base == null)
               throw new RuntimeException("Base with name " + packet.getBaseName() + " is not an available head control base");

            Quat4d quaternion = packet.getQuaternion();
            Point3d point = packet.getPoint();

            if (quaternion != null)
            {
               FrameOrientation frameOrientation = new FrameOrientation(frame, quaternion);
               headOrientationControlModule.setOrientationToTrack(frameOrientation, base);
            }

            if (point != null)
            {
               FramePoint pointToTrack = new FramePoint(frame, point);
               headOrientationControlModule.setPointToTrack(pointToTrack, base);
            }
            
            desiredJointForExtendedNeckPitchRangeAngle = packet.getDesiredJointForExtendedNeckPitchRangeAngle();
         }
      }
   }

   public double getDesiredExtendedNeckPitchJointAngle()
   {
      return desiredJointForExtendedNeckPitchRangeAngle;
   }
}
