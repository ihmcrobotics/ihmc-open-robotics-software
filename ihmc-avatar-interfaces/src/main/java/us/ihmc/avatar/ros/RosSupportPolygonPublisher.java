package us.ihmc.avatar.ros;

import java.util.List;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import ihmc_msgs.Point2dRosMessage;
import ihmc_msgs.SupportPolygonRosMessage;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;

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

   public void publish(Point2D[] points, int numberOfVertices)
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

   public void publish(Point2D32[] points, int numberOfVertices)
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
