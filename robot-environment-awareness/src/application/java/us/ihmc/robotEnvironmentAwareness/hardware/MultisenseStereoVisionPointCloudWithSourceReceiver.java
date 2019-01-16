package us.ihmc.robotEnvironmentAwareness.hardware;

import java.awt.Color;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TFloatArrayList;
import scan_to_cloud.PointCloud2WithSource;
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

public class MultisenseStereoVisionPointCloudWithSourceReceiver extends AbstractRosTopicSubscriber<PointCloud2WithSource>
{
   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "stereoVisionPublisherNode");

   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> stereoVisionPublisher;

   public MultisenseStereoVisionPointCloudWithSourceReceiver() throws URISyntaxException, IOException
   {
      super(PointCloud2WithSource._TYPE);
      URI masterURI = new URI("http://10.6.192.14:11311");
      RosMainNode rosMainNode = new RosMainNode(masterURI, "StereoVisionPublisher", true);
      rosMainNode.attachSubscriber("/stereoVisionPointCloudWithSource", this);
      rosMainNode.execute();

      stereoVisionPublisher = ROS2Tools.createPublisher(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
   }

   @Override
   public void onNewMessage(PointCloud2WithSource cloudHolder)
   {
      UnpackedPointCloud pointCloudData = RosPointCloudSubscriber.unpackPointsAndIntensities(cloudHolder.getCloud());

      int numberOfPoints = pointCloudData.getPoints().length;
      Point3D[] pointCloud = pointCloudData.getPoints();
      Color[] colors = pointCloudData.getPointColors();

      long timestamp = cloudHolder.getCloud().getHeader().getStamp().totalNsecs();

      TFloatArrayList pointCloudBuffer = new TFloatArrayList();

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D scanPoint = pointCloud[i];

         pointCloudBuffer.add((float) scanPoint.getX());
         pointCloudBuffer.add((float) scanPoint.getY());
         pointCloudBuffer.add((float) scanPoint.getZ());
      }

      int[] colorsInteger = new int[numberOfPoints];

      for (int i = 0; i < colors.length; i++)
         colorsInteger[i] = colors[i].getRGB();

      StereoVisionPointCloudMessage stereoVisionMessage = MessageTools.createStereoVisionPointCloudMessage(timestamp, pointCloudBuffer.toArray(),
                                                                                                           colorsInteger);

      stereoVisionPublisher.publish(stereoVisionMessage);
   }

   public static void main(String[] args) throws URISyntaxException, IOException
   {
      new MultisenseStereoVisionPointCloudWithSourceReceiver();
   }
}
