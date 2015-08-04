package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Handstep;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.manipulation.HandstepPacket;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;


public class DesiredHandstepProvider implements PacketConsumer<HandstepPacket>, HandstepProvider
{
   private final AtomicReference<HandstepPacket> handstepPacket = new AtomicReference<HandstepPacket>();
   private final FullRobotModel fullRobotModel;

   public DesiredHandstepProvider(FullRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
   }

   public boolean checkForNewHandstep(RobotSide robotSide)
   {
      return checkForNewHandstep() == robotSide;
   }

   public RobotSide checkForNewHandstep()
   {
      HandstepPacket handstepPacket2 = handstepPacket.get();
      if (handstepPacket2 == null)
         return null;

      return handstepPacket2.getRobotSide();
   }

   public Handstep getDesiredHandstep(RobotSide robotSide)
   {
      HandstepPacket object = handstepPacket.getAndSet(null);
      if (object == null)
         return null;

      if (robotSide != object.getRobotSide())
         return null;

      Point3d location = object.getLocation();
      Quat4d orientation = object.getOrientation();
      Vector3d surfaceNormal = object.getSurfaceNormal();

      FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame(), location, orientation);
      FrameVector surfaceNormalFrameVector = new FrameVector(ReferenceFrame.getWorldFrame(), surfaceNormal);
      Handstep desiredHandstep = new Handstep(robotSide, fullRobotModel.getHand(robotSide), framePose, surfaceNormalFrameVector, object.swingTrajectoryTime);

      return desiredHandstep;
   }

   public void receivedPacket(HandstepPacket object)
   {
      handstepPacket.set(object);
   }

}
