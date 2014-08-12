package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Handstep;
import us.ihmc.commonWalkingControlModules.packets.HandstepPacket;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectConsumer;


public class DesiredHandstepProvider implements ObjectConsumer<HandstepPacket>, HandstepProvider
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

      FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame(), location, orientation);
      Handstep desiredHandstep = new Handstep(fullRobotModel.getHand(robotSide), framePose);

      return desiredHandstep;
   }

   public void consumeObject(HandstepPacket object)
   {
      handstepPacket.set(object);
   }

}
