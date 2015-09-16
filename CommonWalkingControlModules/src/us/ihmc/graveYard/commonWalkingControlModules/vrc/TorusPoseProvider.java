package us.ihmc.graveYard.commonWalkingControlModules.vrc;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.TorusPosePacket;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * @author twan
 *         Date: 5/14/13
 */
public class TorusPoseProvider implements PacketConsumer<TorusPosePacket>
{

   private final AtomicReference<TorusPosePacket> torusPosePacket = new AtomicReference<TorusPosePacket>();

   private FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame());
   private double fingerHoleRadius;

   public TorusPoseProvider()
   {

   }

   public void receivedPacket(TorusPosePacket object)
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
