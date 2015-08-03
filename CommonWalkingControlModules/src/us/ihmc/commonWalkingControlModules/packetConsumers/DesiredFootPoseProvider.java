package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.walking.FootPosePacket;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/22/13
 * Time: 11:45 AM
 * To change this template use File | Settings | File Templates.
 */

public class DesiredFootPoseProvider implements PacketConsumer<FootPosePacket>, FootPoseProvider
{
   private final AtomicReference<FootPosePacket> footPosePacket = new AtomicReference<FootPosePacket>();

   /**
    * -1 => null
    *  0 => RobotSide.LEFT
    *  1 => RobotSide.RIGHT
    */
   private final AtomicInteger footPoseSide = new AtomicInteger(-1);
   private final FramePose desiredFootPose = new FramePose();
   private double trajectoryTime = Double.NaN;
   private final double defaultTrajectoryTime;

   private final GlobalDataProducer globalDataProducer;

   public DesiredFootPoseProvider(double defaultTrajectoryTime, GlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;
      this.defaultTrajectoryTime = defaultTrajectoryTime;
   }

   @Override
   public boolean checkForNewPose(RobotSide robotSide)
   {
      return checkForNewPose() == robotSide;
   }

   @Override
   public RobotSide checkForNewPose()
   {
      if (footPoseSide.get() == -1)
         return null;
      else
         return footPoseSide.get() == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
   }

   @Override
   public void clear()
   {
      footPosePacket.set(null);
      footPoseSide.set(-1);
   }

   @Override
   public FramePose getDesiredFootPose(RobotSide robotSide)
   {
      FootPosePacket object = footPosePacket.getAndSet(null);
      footPoseSide.set(-1);

      if (object != null)
         desiredFootPose.setPoseIncludingFrame(ReferenceFrame.getWorldFrame(), object.getPosition(), object.getOrientation());

      return desiredFootPose;
   }

   @Override
   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   @Override
   public void receivedPacket(FootPosePacket object)
   {
      if (globalDataProducer != null)
      {
         String errorMessage = PacketValidityChecker.validateFootPosePacket(object);
         if (errorMessage != null)
         {
            globalDataProducer.notifyInvalidPacketReceived(FootPosePacket.class, errorMessage);
            return;
         }
      }
      
      RobotSide robotSide = object.getRobotSide();
      footPoseSide.set(robotSide == RobotSide.LEFT ? 0 : 1);
      trajectoryTime = object.getTrajectoryTime();
      if (Double.isNaN(trajectoryTime) || trajectoryTime < 0.01) 
         trajectoryTime = defaultTrajectoryTime;
      footPosePacket.set(object);
   }
}
