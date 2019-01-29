package us.ihmc.robotEnvironmentAwareness.hardware;

import java.awt.Color;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TFloatArrayList;
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
   private static final int MAX_NUMBER_OF_POINTS = 200000;

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

      List<Point3D> pointCloudToPublish = Arrays.stream(pointCloud).collect(Collectors.toList());
      List<Color> colorsToPublish = Arrays.stream(colors).collect(Collectors.toList());

      Random random = new Random();
      while (pointCloudToPublish.size() > MAX_NUMBER_OF_POINTS)
      {
         int indexToRemove = random.nextInt(pointCloudToPublish.size());
         pointCloudToPublish.remove(indexToRemove);
         colorsToPublish.remove(indexToRemove);
      }

      int numberOfPoints = pointCloudToPublish.size();

      long timestamp = cloudHolder.getHeader().getStamp().totalNsecs();
      TFloatArrayList pointCloudBuffer = new TFloatArrayList();

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D scanPoint = pointCloudToPublish.get(i);

         pointCloudBuffer.add((float) scanPoint.getX());
         pointCloudBuffer.add((float) scanPoint.getY());
         pointCloudBuffer.add((float) scanPoint.getZ());
      }

      int[] colorsInteger = new int[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
         colorsInteger[i] = colorsToPublish.get(i).getRGB();

      StereoVisionPointCloudMessage stereoVisionMessage = MessageTools.createStereoVisionPointCloudMessage(timestamp, pointCloudBuffer.toArray(),
                                                                                                           colorsInteger);

      stereoVisionPublisher.publish(stereoVisionMessage);
   }

   public static void main(String[] args) throws URISyntaxException
   {
      new MultisenseStereoVisionPointCloudReceiver();
   }
}
