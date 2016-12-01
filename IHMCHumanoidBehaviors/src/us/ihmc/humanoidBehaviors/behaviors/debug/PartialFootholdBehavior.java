package us.ihmc.humanoidBehaviors.behaviors.debug;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.robotSide.RobotSide;

public class PartialFootholdBehavior extends AbstractBehavior
{

   public PartialFootholdBehavior(CommunicationBridgeInterface communicationBridge)
   {
      super(communicationBridge);
   }

   @Override
   public void doControl()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorEntered()
   {
      FootstepDataListMessage message = new FootstepDataListMessage(1.5, 0.5);
      RobotSide side = RobotSide.LEFT;

      for (int i = 0; i < 10; i++)
      {
         double y = side == RobotSide.LEFT ? 0.15 : -0.15;
         Point3d location = new Point3d(0.3 * i, y, 0.0);

         ArrayList<Point2d> contactPoints = new ArrayList<>();
         if (i == 0 || i == 3 || i == 7 || i == 9)
            contactPoints = null;
         else if (i == 1 || i == 8 || i == 5)
         {
            contactPoints.add(new Point2d(0.05, 0.05));
            contactPoints.add(new Point2d(-0.05, 0.05));
            contactPoints.add(new Point2d(-0.05, -0.05));
            contactPoints.add(new Point2d(0.05, -0.05));
         }
         else if (i == 2 || i == 4 || i == 6)
         {
            contactPoints.add(new Point2d(0.1, 0.05));
            contactPoints.add(new Point2d(-0.1, 0.05));
            contactPoints.add(new Point2d(0.1, 0.0));
            contactPoints.add(new Point2d(-0.1, 0.0));
         }

         FootstepDataMessage footstepData = new FootstepDataMessage(side, location, new Quat4d(0.0, 0.0, 0.0, 1.0), contactPoints);
         footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         message.add(footstepData);
         side = side.getOppositeSide();
      }

      sendPacket(message);
   }

   @Override
   public void onBehaviorAborted()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorPaused()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorResumed()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public boolean isDone()
   {
      return true;
   }

}
