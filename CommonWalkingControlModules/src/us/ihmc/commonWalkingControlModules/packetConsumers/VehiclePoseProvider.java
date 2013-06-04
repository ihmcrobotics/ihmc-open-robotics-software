package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.commonWalkingControlModules.packets.VehiclePosePacket;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

public class VehiclePoseProvider implements ObjectConsumer<VehiclePosePacket>
{
   private final Object synchronizationObject = new Object();
   private FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame());
   private boolean hasNewPose;

   public void consumeObject(VehiclePosePacket object)
   {
      synchronized (synchronizationObject)
      {
         framePose = new FramePose(ReferenceFrame.getWorldFrame(), object.getPosition(), object.getOrientation());
         hasNewPose = true;
      }
   }

   public boolean checkForNewPose()
   {
      synchronized (synchronizationObject)
      {
         return hasNewPose;
      }
   }

   public FramePoint getPosition()
   {
      synchronized (synchronizationObject)
      {
         return framePose.getPostionCopy();
      }
   }

   public FrameOrientation getOrientation()
   {
      synchronized (synchronizationObject)
      {
         return framePose.getOrientationCopy();
      }
   }

   public FramePose getFramePose()
   {
      synchronized (synchronizationObject)
      {
         hasNewPose = false;

         return new FramePose(framePose);
      }
   }
}

