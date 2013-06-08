package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.commonWalkingControlModules.packets.VehiclePosePacket;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

public class VehiclePoseProvider implements ObjectConsumer<VehiclePosePacket>
{
   private FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame());
   private boolean hasNewPose;

   public synchronized void consumeObject(VehiclePosePacket object)
   {
      framePose = new FramePose(ReferenceFrame.getWorldFrame(), object.getPosition(), object.getOrientation());
      hasNewPose = true;
   }

   public synchronized boolean checkForNewPose()
   {
      return hasNewPose;
   }

   public synchronized FramePoint getPosition()
   {
      return framePose.getPostionCopy();
   }

   public synchronized FrameOrientation getOrientation()
   {
      return framePose.getOrientationCopy();
   }

   public synchronized FramePose getFramePose()
   {
      hasNewPose = false;

      return new FramePose(framePose);
   }
}
