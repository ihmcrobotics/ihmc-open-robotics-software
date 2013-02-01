package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.HashMap;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.commonWalkingControlModules.controlModules.HeadOrientationControlModule;
import us.ihmc.utilities.io.streamingData.AbstractStreamingDataConsumer;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

/**
 * User: Matt
 * Date: 1/28/13
 */
public class DesiredHeadOrientationProvider
{
   private static final boolean DEBUG = false;
   private HeadOrientationControlModule headOrientationControlModule;

   private AbstractStreamingDataConsumer<HeadOrientationPacket> headOrientationPacketConsumer;
   private final HashMap<String, ReferenceFrame> availableHeadControlFrames = new HashMap<String, ReferenceFrame>();
   private final HashMap<String, RigidBody> availableBases = new HashMap<String, RigidBody>();

   public DesiredHeadOrientationProvider(long headOrientationControlIdentifier)
   {
      headOrientationPacketConsumer = new HeadOrientationPacketConsumer(headOrientationControlIdentifier);
   }

   public AbstractStreamingDataConsumer<HeadOrientationPacket> getHeadOrientationPacketConsumer()
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

   private class HeadOrientationPacketConsumer extends AbstractStreamingDataConsumer<HeadOrientationPacket>
   {
      public HeadOrientationPacketConsumer(long objectIdentifier)
      {
         super(objectIdentifier, HeadOrientationPacket.class);
      }

      protected void processPacket(HeadOrientationPacket packet)
      {
         if (DEBUG)
         {
            System.out.println(packet);
         }

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

            boolean hasBeenSet = false;
            if (quaternion != null)
            {
               assert !hasBeenSet;
               FrameOrientation frameOrientation = new FrameOrientation(frame, quaternion);
               headOrientationControlModule.setOrientationToTrack(frameOrientation, base);
            }

            if (point != null)
            {
               assert !hasBeenSet;
               FramePoint pointToTrack = new FramePoint(frame, point);
               headOrientationControlModule.setPointToTrack(pointToTrack, base);
            }

            assert hasBeenSet;
         }
      }
   }
}
