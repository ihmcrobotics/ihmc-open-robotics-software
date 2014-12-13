package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packets.walking.FootPosePacket;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/22/13
 * Time: 11:45 AM
 * To change this template use File | Settings | File Templates.
 */

public class DesiredFootPoseProvider implements ObjectConsumer<FootPosePacket>, FootPoseProvider
{
   private final AtomicReference<FootPosePacket> footPosePacket = new AtomicReference<FootPosePacket>();

   /**
    * -1 => null
    *  0 => RobotSide.LEFT
    *  1 => RobotSide.RIGHT
    */
   private final AtomicInteger footPoseSide = new AtomicInteger(-1);
   private final FramePose desiredFootPose = new FramePose();

   public DesiredFootPoseProvider()
   {
   }

   public boolean checkForNewPose(RobotSide robotSide)
   {
      return checkForNewPose() == robotSide;
   }

   public RobotSide checkForNewPose()
   {
      if (footPoseSide.get() == -1)
         return null;
      else
         return footPoseSide.get() == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
   }

   public FramePose getDesiredFootPose(RobotSide robotSide)
   {
      FootPosePacket object = footPosePacket.getAndSet(null);
      footPoseSide.set(-1);

      if (object != null)
         desiredFootPose.setPoseIncludingFrame(ReferenceFrame.getWorldFrame(), object.getPosition(), object.getOrientation());

      return desiredFootPose;
   }

   public void consumeObject(FootPosePacket object)
   {
      RobotSide robotSide = object.getRobotSide();
      footPoseSide.set(robotSide == RobotSide.LEFT ? 0 : 1);

      footPosePacket.set(object);
   }
}
