package us.ihmc.avatar.ros;

import ihmc_msgs.Point2dRosMessage;
import ihmc_msgs.SupportPolygonRosMessage;
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
public class RosSupportPolygonPublisher extends RosTopicPublisher<SupportPolygonRosMessage>
{
   private final MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();

   public RosSupportPolygonPublisher(boolean latched)
   {
      super(SupportPolygonRosMessage._TYPE, latched);
   }

   public void publish(Point2d[] points, int numberOfVertices)
   {
      SupportPolygonRosMessage message = getMessage();
      message.setNumberOfVertices(numberOfVertices);
      List<Point2dRosMessage> vertices = message.getVertices();
      vertices.clear();
      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2dRosMessage point2d = messageFactory.newFromType(Point2dRosMessage._TYPE);
         point2d.setX((float) points[i].getX());
         point2d.setY((float) points[i].getY());
         vertices.add(point2d);
      }
      publish(message);
   }

   public void publish(Point2f[] points, int numberOfVertices)
   {
      SupportPolygonRosMessage message = getMessage();
      message.setNumberOfVertices(numberOfVertices);
      List<Point2dRosMessage> vertices = message.getVertices();
      vertices.clear();
      for (int i = 0; i < numberOfVertices; i++)
      {

         Point2dRosMessage point2d = messageFactory.newFromType(Point2dRosMessage._TYPE);
         point2d.setX(points[i].getX());
         point2d.setY(points[i].getY());
         vertices.add(point2d);

      }
      publish(message);
   }
}
