package us.ihmc.ihmcPerception.faceDetection;

import people_msgs.PositionMeasurement;
import people_msgs.PositionMeasurementArray;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import javax.vecmath.Point3d;

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
      PositionMeasurement[] people = (PositionMeasurement[]) message.getPeople().toArray();

      String[] ids = new String[people.length];
      Point3d[] positions = new Point3d[people.length];

      for(int i = 0; i < people.length; i++)
      {
         PositionMeasurement person = people[i];
         ids[i] = person.getObjectId();
         positions[i] = new Point3d(person.getPos().getX(), person.getPos().getY(), person.getPos().getZ());
      }

      DetectedFacesPacket detectedFaces = new DetectedFacesPacket(ids, positions);

   }
}
