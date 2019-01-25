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
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
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

   private static final Point3D translationToFixedFrame = new Point3D();
   private static final Quaternion rotationToFixedFrame = new Quaternion(-0.5, 0.5, -0.5, 0.5);
   private static final RigidBodyTransform transformToFixedFrame = new RigidBodyTransform(rotationToFixedFrame, translationToFixedFrame);

   public MultisenseStereoVisionPointCloudReceiver() throws URISyntaxException
   {
      super(PointCloud2._TYPE);
      URI masterURI = new URI("http://10.6.192.14:11311");
      RosMainNode rosMainNode = new RosMainNode(masterURI, "StereoVisionPublisher", true);
      rosMainNode.attachSubscriber("/multisense/image_points2_color", this);

      rosMainNode.execute();

      stereoVisionPublisher = ROS2Tools.createPublisher(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
   }

   @Override
   public void onNewMessage(PointCloud2 cloudHolder)
   {
      UnpackedPointCloud pointCloudData = RosPointCloudSubscriber.unpackPointsAndIntensities(cloudHolder);

      Point3D[] pointCloud = transformPointCloud(pointCloudData.getPoints());
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

   private Point3D[] transformPointCloud(Point3D[] pointCloud)
   {
      Point3D[] transformedPointCloud = new Point3D[pointCloud.length];

      for (int i = 0; i < pointCloud.length; i++)
      {
         transformedPointCloud[i] = new Point3D();
         transformToFixedFrame.transform(pointCloud[i], transformedPointCloud[i]);
      }

      return transformedPointCloud;
   }
}
