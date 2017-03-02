package us.ihmc.utilities.ros.publisher;

import ihmc_msgs.Point2dRosMessage;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class RosPoint2dPublisher extends RosTopicPublisher<Point2dRosMessage>
{
   public RosPoint2dPublisher(boolean latched)
   {
      super(Point2dRosMessage._TYPE, latched);
   }

   public void publish(float x, float y)
   {
      Point2dRosMessage message = getMessage();
      message.setX(x);
      message.setY(y);
      publish(message);
   }

   public void publish(Point2DBasics point2d)
   {
      publish((float) point2d.getX(), (float) point2d.getY());
   }
}
