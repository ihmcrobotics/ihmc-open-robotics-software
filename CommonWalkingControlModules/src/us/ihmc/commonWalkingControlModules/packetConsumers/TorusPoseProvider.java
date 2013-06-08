package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.commonWalkingControlModules.packets.TorusPosePacket;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;

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

   public synchronized void consumeObject(TorusPosePacket object)
   {
      synchronized (synchronizationObject)
      {
         framePose = new FramePose(ReferenceFrame.getWorldFrame(), object.getPosition(), object.getOrientation());
         fingerHoleRadius = object.getFingerHoleRadius();
         hasNewPose = true;
      }
   }

   public synchronized boolean checkForNewPose()
   {
      synchronized (synchronizationObject)
      {
         return hasNewPose;
      }
   }

   public synchronized double getFingerHoleRadius()
   {
      synchronized (synchronizationObject)
      {
         hasNewPose = false;

         return fingerHoleRadius;
      }
   }

   public synchronized FramePose getFramePose()
   {
      synchronized (synchronizationObject)
      {
         hasNewPose = false;

         return new FramePose(framePose);
      }
   }
}
