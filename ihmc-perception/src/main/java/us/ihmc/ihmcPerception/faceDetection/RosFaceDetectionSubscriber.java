package us.ihmc.ihmcPerception.faceDetection;

import people_msgs.PositionMeasurement;
import people_msgs.PositionMeasurementArray;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.DetectedFacesPacket;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;

public class RosFaceDetectionSubscriber extends AbstractRosTopicSubscriber<PositionMeasurementArray>
{
   private final FramePoint3D framePoint = new FramePoint3D();

   private final PacketCommunicator packetCommunicator;
   private final ReferenceFrame cameraFrame;

   public RosFaceDetectionSubscriber(String topicName, RosMainNode node, PacketCommunicator packetCommunicator, ReferenceFrame cameraFrame)
   {
      super(PositionMeasurementArray._TYPE);

      this.packetCommunicator = packetCommunicator;
      this.cameraFrame = cameraFrame;

      node.attachSubscriber(topicName, this);
   }

   @Override
   public void onNewMessage(PositionMeasurementArray message)
   {
      Object[] people = message.getPeople().toArray();

      String[] ids = new String[people.length];
      Point3D[] positions = new Point3D[people.length];

      for(int i = 0; i < people.length; i++)
      {
         PositionMeasurement person = (PositionMeasurement) people[i];
         ids[i] = String.valueOf(i);

         framePoint.setIncludingFrame(cameraFrame, person.getPos().getX(), person.getPos().getY(), person.getPos().getZ());
         framePoint.changeFrame(ReferenceFrame.getWorldFrame());
         positions[i] = new Point3D(framePoint);
      }

      DetectedFacesPacket detectedFaces = new DetectedFacesPacket(ids, positions);
      packetCommunicator.send(detectedFaces);
   }
}
