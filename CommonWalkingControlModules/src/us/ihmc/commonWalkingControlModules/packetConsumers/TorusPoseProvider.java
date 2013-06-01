package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.commonWalkingControlModules.packets.TorusPosePacket;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;

/**
 * @author twan
 *         Date: 5/14/13
 */
public class TorusPoseProvider implements ObjectConsumer<TorusPosePacket>
{
   private final Object synchronizationObject = new Object();
   private FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame());
   private double fingerHoleRadius;
   private boolean hasNewPose;

   public TorusPoseProvider()
   {

   }

   public void consumeObject(TorusPosePacket object)
   {
      synchronized (synchronizationObject)
      {
         framePose = new FramePose(ReferenceFrame.getWorldFrame(), object.getPosition(), object.getOrientation());
         fingerHoleRadius = object.getFingerHoleRadius();
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

   public double getFingerHoleRadius()
   {
      synchronized (synchronizationObject)
      {
         hasNewPose = false;

         return fingerHoleRadius;
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
