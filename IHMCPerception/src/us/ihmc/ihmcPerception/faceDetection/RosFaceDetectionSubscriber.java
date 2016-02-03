package us.ihmc.ihmcPerception.faceDetection;

import people_msgs.PositionMeasurement;
import people_msgs.PositionMeasurementArray;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import javax.vecmath.Point3d;

public class RosFaceDetectionSubscriber extends AbstractRosTopicSubscriber<PositionMeasurementArray>
{
   private final PacketCommunicator packetCommunicator;

   public RosFaceDetectionSubscriber(String topicName, RosMainNode node, PacketCommunicator packetCommunicator)
   {
      super(PositionMeasurementArray._TYPE);

      this.packetCommunicator = packetCommunicator;

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
      packetCommunicator.send(detectedFaces);
   }
}
