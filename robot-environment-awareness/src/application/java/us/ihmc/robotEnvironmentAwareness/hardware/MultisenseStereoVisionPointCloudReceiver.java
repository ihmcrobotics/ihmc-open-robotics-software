package us.ihmc.robotEnvironmentAwareness.hardware;

import java.awt.Color;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Random;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.PointCloud2;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;

public class MultisenseStereoVisionPointCloudReceiver extends AbstractRosTopicSubscriber<PointCloud2>
{
   private static final int MAX_NUMBER_OF_POINTS = 100000;

   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "stereoVisionPublisherNode");

   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> stereoVisionPublisher;

   public MultisenseStereoVisionPointCloudReceiver() throws URISyntaxException
   {
      super(PointCloud2._TYPE);
      URI masterURI = new URI("http://10.6.192.14:11311");
      RosMainNode rosMainNode = new RosMainNode(masterURI, "StereoVisionPublisher", true);
      rosMainNode.attachSubscriber("/multisense/image_points2_color_world", this);

      rosMainNode.execute();

      stereoVisionPublisher = ROS2Tools.createPublisher(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
   }

   @Override
   public void onNewMessage(PointCloud2 cloudHolder)
   {
      UnpackedPointCloud pointCloudData = RosPointCloudSubscriber.unpackPointsAndIntensities(cloudHolder);

      Point3D[] pointCloud = pointCloudData.getPoints();
      Color[] colors = pointCloudData.getPointColors();

      Random random = new Random();
      int numberOfPoints = pointCloud.length;

      while (numberOfPoints > MAX_NUMBER_OF_POINTS)
      {
         int indexToRemove = random.nextInt(numberOfPoints);
         int lastIndex = numberOfPoints - 1;

         pointCloud[indexToRemove] = pointCloud[lastIndex];
         colors[indexToRemove] = colors[lastIndex];

         numberOfPoints--;
      }

      long timestamp = cloudHolder.getHeader().getStamp().totalNsecs();
      float[] pointCloudBuffer = new float[3 * numberOfPoints];
      int[] colorsInteger = new int[numberOfPoints];

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D scanPoint = pointCloud[i];

         pointCloudBuffer[3 * i + 0] = (float) scanPoint.getX();
         pointCloudBuffer[3 * i + 1] = (float) scanPoint.getY();
         pointCloudBuffer[3 * i + 2] = (float) scanPoint.getZ();

         colorsInteger[i] = colors[i].getRGB();
      }

      StereoVisionPointCloudMessage stereoVisionMessage = MessageTools.createStereoVisionPointCloudMessage(timestamp, pointCloudBuffer, colorsInteger);

      stereoVisionPublisher.publish(stereoVisionMessage);
   }

   public static void main(String[] args) throws URISyntaxException
   {
      new MultisenseStereoVisionPointCloudReceiver();
   }
}
