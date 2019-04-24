package us.ihmc.humanoidBehaviors.behaviors.debug;

import java.util.ArrayList;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;

public class PartialFootholdBehavior extends AbstractBehavior
{
   private final IHMCROS2Publisher<FootstepDataListMessage> publisher;

   public PartialFootholdBehavior(String robotName, Ros2Node ros2Node)
   {
      super(robotName, ros2Node);
      publisher = createPublisherForController(FootstepDataListMessage.class);
   }

   @Override
   public void doControl()
   {

   }

   @Override
   public void onBehaviorEntered()
   {
      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(1.5, 0.5);
      RobotSide side = RobotSide.LEFT;

      for (int i = 0; i < 10; i++)
      {
         double y = side == RobotSide.LEFT ? 0.15 : -0.15;
         Point3D location = new Point3D(0.3 * i, y, 0.0);

         ArrayList<Point2D> contactPoints = new ArrayList<>();
         if (i == 0 || i == 3 || i == 7 || i == 9)
            contactPoints = null;
         else if (i == 1 || i == 8 || i == 5)
         {
            contactPoints.add(new Point2D(0.05, 0.05));
            contactPoints.add(new Point2D(-0.05, 0.05));
            contactPoints.add(new Point2D(-0.05, -0.05));
            contactPoints.add(new Point2D(0.05, -0.05));
         }
         else if (i == 2 || i == 4 || i == 6)
         {
            contactPoints.add(new Point2D(0.1, 0.05));
            contactPoints.add(new Point2D(-0.1, 0.05));
            contactPoints.add(new Point2D(0.1, 0.0));
            contactPoints.add(new Point2D(-0.1, 0.0));
         }

         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(side, location, new Quaternion(0.0, 0.0, 0.0, 1.0), contactPoints);
         message.getFootstepDataList().add().set(footstepData);
         side = side.getOppositeSide();
      }

      publisher.publish(message);
   }

   @Override
   public void onBehaviorAborted()
   {

   }

   @Override
   public void onBehaviorPaused()
   {

   }

   @Override
   public void onBehaviorResumed()
   {

   }

   @Override
   public void onBehaviorExited()
   {

   }

   @Override
   public boolean isDone(double timeinState)
   {
      return true;
   }

}
