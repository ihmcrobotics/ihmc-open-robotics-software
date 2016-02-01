package us.ihmc.utilities.ros.subscriber;

import geometry_msgs.Point;
import people_msgs.PositionMeasurement;
import people_msgs.PositionMeasurementArray;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.util.Iterator;
import java.util.List;

public class RosFaceDetectionSubscriber extends AbstractRosTopicSubscriber<PositionMeasurementArray>
{
   public RosFaceDetectionSubscriber(String topicName, RosMainNode node)
   {
      super(PositionMeasurementArray._TYPE);

      node.attachSubscriber(topicName, this);
   }

   @Override
   public void onNewMessage(PositionMeasurementArray message)
   {
      System.out.println(message.getPeople().size() + " people detected");

      List<PositionMeasurement> people = message.getPeople();
      Iterator<PositionMeasurement> iterator = people.iterator();
      while(iterator.hasNext())
      {
         PositionMeasurement positionMeasurement = iterator.next();
         String id = positionMeasurement.getObjectId();
         Point position = positionMeasurement.getPos();

         System.out.println("Detected face with id: " + id + " at position: (" + position.getX() + "," + position.getY() + "," + position.getZ() + ")");
      }
   }
}
