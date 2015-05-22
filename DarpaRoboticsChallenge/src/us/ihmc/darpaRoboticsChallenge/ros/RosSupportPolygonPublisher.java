package us.ihmc.darpaRoboticsChallenge.ros;

import ihmc_msgs.Point2dMessage;
import ihmc_msgs.SupportPolygonMessage;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;

import javax.vecmath.Point2d;
import javax.vecmath.Point2f;
import java.util.List;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class RosSupportPolygonPublisher extends RosTopicPublisher<SupportPolygonMessage>
{
   private final MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();

   public RosSupportPolygonPublisher(boolean latched)
   {
      super(SupportPolygonMessage._TYPE, latched);
   }

   public void publish(Point2d[] points)
   {
      SupportPolygonMessage message = getMessage();
      List<Point2dMessage> vertices = message.getVertices();
      vertices.clear();
      int length = 0;
      for (Point2d point : points)
      {
         if(!isZeroPoint(point))
         {
            Point2dMessage point2d = messageFactory.newFromType(Point2dMessage._TYPE);
            point2d.setX((float) point.x);
            point2d.setY((float) point.y);
            vertices.add(point2d);
            length++;
         }
      }
      message.setNumberOfVertices(length);
      publish(message);
   }

   public void publish(Point2f[] points)
   {
      SupportPolygonMessage message = getMessage();
      List<Point2dMessage> vertices = message.getVertices();
      vertices.clear();
      int length = 0;
      for (Point2f point : points)
      {
         if(!isZeroPoint(point))
         {
            Point2dMessage point2d = messageFactory.newFromType(Point2dMessage._TYPE);
            point2d.setX(point.x);
            point2d.setY(point.y);
            vertices.add(point2d);
            length++;
         }
      }
      message.setNumberOfVertices(length);
      publish(message);
   }

   private boolean isZeroPoint(Point2d point)
   {
      return point.x == 0.0 && point.y == 0.0;
   }

   private boolean isZeroPoint(Point2f point)
   {
      return point.x == 0.0f && point.y == 0.0f;
   }
}
