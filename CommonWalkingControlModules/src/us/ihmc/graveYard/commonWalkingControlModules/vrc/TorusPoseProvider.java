package us.ihmc.graveYard.commonWalkingControlModules.vrc;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packets.manipulation.TorusPosePacket;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

/**
 * @author twan
 *         Date: 5/14/13
 */
public class TorusPoseProvider implements ObjectConsumer<TorusPosePacket>
{

   private final AtomicReference<TorusPosePacket> torusPosePacket = new AtomicReference<TorusPosePacket>();

   private FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame());
   private double fingerHoleRadius;

   public TorusPoseProvider()
   {

   }

   public void consumeObject(TorusPosePacket object)
   {
      torusPosePacket.set(object);
   }

   public boolean checkForNewPose()
   {
      TorusPosePacket object = torusPosePacket.getAndSet(null);
      if (object != null)
      {
         framePose = new FramePose(ReferenceFrame.getWorldFrame(), object.getPosition(), object.getOrientation());
         fingerHoleRadius = object.getFingerHoleRadius();
         return true;
      }
      else
      {
         return false;
      }
   }

   public double getFingerHoleRadius()
   {
      return fingerHoleRadius;
   }

   public FramePose getFramePose()
   {
      return new FramePose(framePose);
   }
}
