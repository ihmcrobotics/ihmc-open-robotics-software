package us.ihmc.darpaRoboticsChallenge.ros;

import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point2f;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import ihmc_msgs.Point2dMessage;
import ihmc_msgs.SupportPolygonMessage;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;

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

   public void publish(Point2d[] points, int numberOfVertices)
   {
      SupportPolygonMessage message = getMessage();
      message.setNumberOfVertices(numberOfVertices);
      List<Point2dMessage> vertices = message.getVertices();
      vertices.clear();
      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2dMessage point2d = messageFactory.newFromType(Point2dMessage._TYPE);
         point2d.setX((float) points[i].x);
         point2d.setY((float) points[i].y);
         vertices.add(point2d);
      }
      publish(message);
   }

   public void publish(Point2f[] points, int numberOfVertices)
   {
      SupportPolygonMessage message = getMessage();
      message.setNumberOfVertices(numberOfVertices);
      List<Point2dMessage> vertices = message.getVertices();
      vertices.clear();
      for (int i = 0; i < numberOfVertices; i++)
      {

         Point2dMessage point2d = messageFactory.newFromType(Point2dMessage._TYPE);
         point2d.setX(points[i].x);
         point2d.setY(points[i].y);
         vertices.add(point2d);

      }
      publish(message);
   }
}
