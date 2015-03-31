package us.ihmc.darpaRoboticsChallenge.ros;

import java.awt.image.BufferedImage;

import org.ros.message.Time;

import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packets.LocalVideoPacket;
import us.ihmc.communication.packets.sensing.IntrinsicCameraParametersPacket;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosCameraInfoPublisher;
import us.ihmc.utilities.ros.publisher.RosImagePublisher;

public class RosSCSCameraPublisher implements ObjectConsumer<LocalVideoPacket>
{
   private final RosImagePublisher[] cameraPublisher;
   private final RosMainNode rosMainNode;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final DRCRobotCameraParameters[] cameraParameters;
   private final RosCameraInfoPublisher[] cameraInfoPublishers;

   private final int nSensors;

   public RosSCSCameraPublisher(ObjectCommunicator localObjectCommunicator, RosMainNode rosMainNode, PPSTimestampOffsetProvider ppsTimestampOffsetProvider,
         DRCRobotCameraParameters[] cameraParameters)
   {
      nSensors = cameraParameters.length;
      this.rosMainNode = rosMainNode;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.cameraParameters = cameraParameters;

      cameraPublisher = new RosImagePublisher[nSensors];
      cameraInfoPublishers = new RosCameraInfoPublisher[nSensors];

      for (int sensorId = 0; sensorId < nSensors; sensorId++)
      {
         cameraPublisher[sensorId] = new RosImagePublisher();
         String imageTopic = cameraParameters[sensorId].getRosTopic();
         rosMainNode.attachPublisher(imageTopic, cameraPublisher[sensorId]);

         if (cameraParameters[sensorId].getRosCameraInfoTopicName() != null && cameraParameters[sensorId].getRosCameraInfoTopicName() != "")
         {
            String infoTopic = cameraParameters[sensorId].getRosCameraInfoTopicName();
            cameraInfoPublishers[sensorId] = new RosCameraInfoPublisher();
            rosMainNode.attachPublisher(infoTopic, cameraInfoPublishers[sensorId]);
         }
      }

      localObjectCommunicator.attachListener(LocalVideoPacket.class, this);
   }

   @Override
   public void consumeObject(LocalVideoPacket object)
   {
      if (rosMainNode.isStarted())
      {
         //XXX: SENSOR ID DOES NOT EXIST! THIS IS SOOOOOO WRONG
         int sensorId = 0;
         long timestamp = ppsTimestampOffsetProvider.adjustRobotTimeStampToRosClock(object.getTimeStamp());
         Time time = Time.fromNano(timestamp);
         String frameId = cameraParameters[sensorId].getPoseFrameForSdf();
         cameraPublisher[sensorId].publish(frameId, object.getImage(), time);
         sendIntrinsicPacket(object, sensorId, frameId, time);
      }
   }

   public void sendIntrinsicPacket(LocalVideoPacket videoObject, int sensorId, String frameId, Time time)
   {
      if (cameraInfoPublishers[sensorId] == null)
      {
         return;
      }

      BufferedImage img = videoObject.getImage();
      double f = videoObject.getImage().getWidth() / 2 / Math.tan(videoObject.getFieldOfView() / 2);
      IntrinsicCameraParametersPacket packet = new IntrinsicCameraParametersPacket(f, f, 0, (img.getWidth() - 1) / 2f, (img.getHeight() - 1) / 2f,
            img.getWidth(), img.getHeight());
      cameraInfoPublishers[sensorId].publish(frameId, packet, time);
   }
}
