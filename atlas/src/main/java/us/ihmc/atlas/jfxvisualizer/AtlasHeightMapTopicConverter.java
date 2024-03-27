package us.ihmc.atlas.jfxvisualizer;

import perception_msgs.msg.dds.PlanarRegionsListMessage;
import map_sense.RawGPUPlanarRegionList;
import org.apache.commons.lang.mutable.MutableInt;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.LittleEndianHeapChannelBuffer;
import org.ros.message.Duration;
import org.ros.message.Time;
import sensor_msgs.PointCloud2;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.perception.depthData.PointCloudData;
import us.ihmc.avatar.ros.RosTfPublisher;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.updaters.GPUPlanarRegionUpdater;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.publisher.RosClockPublisher;
import us.ihmc.utilities.ros.publisher.RosPointCloudPublisher;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.types.PointType;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Class for various ROS1 topic management related to height maps.
 */
public class AtlasHeightMapTopicConverter
{
   private static boolean debug = false;
   private static final String atlasPelvisFrame = "pelvis";
   private static final String atlasOusterFrame = "ouster";

   public AtlasHeightMapTopicConverter() throws URISyntaxException
   {
      URI rosuri = NetworkParameters.getROSURI();
      System.out.println("ROS MASTER URI " + rosuri);
//      URI rosuri = new URI("http://172.16.66.102:11311");

      RosMainNode rosNode = RosTools.createRosNode(rosuri, "ouster_turtle_sim");
      AtomicReference<PointCloud2> inputPointCloud = new AtomicReference<>();

      MutableInt counter = new MutableInt();
      rosNode.attachSubscriber(RosTools.OUSTER_POINT_CLOUD, new AbstractRosTopicSubscriber<PointCloud2>(PointCloud2._TYPE)
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud)
         {
//            LogTools.info("received point cloud " + counter.getValue());
            counter.increment();
            inputPointCloud.set(pointCloud);
         }
      });

//      setupForTurtleSim(rosNode, inputPointCloud);
//      setupForAtlasSim(rosNode, inputPointCloud);
      setupForRealRobot(rosNode, inputPointCloud);

      rosNode.execute();
   }

   private static void setupForTurtleSim(RosMainNode rosNode, AtomicReference<PointCloud2> inputPointCloud)
   {
      RosPointCloudPublisher publisher = new RosPointCloudPublisher(PointType.XYZI, false);
      rosNode.attachPublisher("/os_cloud_node2/points", publisher);

      RigidBodyTransform approxOusterTransform = new RigidBodyTransform();
      double ousterPitch = Math.toRadians(30.0);
      approxOusterTransform.getRotation().setToPitchOrientation(ousterPitch);
      approxOusterTransform.getTranslation().set(-0.2, 0.0, 1.6);

      MutableInt counter = new MutableInt();
      new Thread(() ->
                 {
                    while (true)
                    {
                       PointCloud2 pointCloud = inputPointCloud.getAndSet(null);
                       if (pointCloud != null)
                       {
                          System.out.println("publishing " + counter.intValue());
                          counter.increment();

                          PointCloudData pointCloudData = new PointCloudData(pointCloud, 1000000, false);
                          pointCloudData.applyTransform(approxOusterTransform);

                          PointCloud2 message = packMessage(publisher.getMessage(), pointCloudData.getPointCloud());

                           pointCloud.getHeader().setFrameId("base_footprint");
//                          message.getHeader().setFrameId("odom");
                          message.getHeader().setStamp(rosNode.getCurrentTime());

                          publisher.publish(message);
                       }

                       ThreadTools.sleep(25);
                    }
                 }).start();
   }

   private static void setupForAtlasSim(RosMainNode ros1Node, AtomicReference<PointCloud2> inputPointCloud)
   {
      RosClockPublisher rosClockPublisher = new RosClockPublisher();
      ros1Node.attachPublisher("/clock", rosClockPublisher);

      new Thread(() ->
                 {
                    while (!ros1Node.isStarted())
                    {
                       LogTools.info("waiting to connect...");
                       ThreadTools.sleep(500);
                    }

                    while (true)
                    {
                       if (inputPointCloud.get() != null)
                       {
                          Time timestamp = inputPointCloud.get().getHeader().getStamp();
                          timestamp.add(new Duration(0.03));
                          rosClockPublisher.publish(timestamp);
                       }
                    }
                 }).start();
   }

   private static void setupForRealRobot(RosMainNode ros1Node, AtomicReference<PointCloud2> inputPointCloud)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "height_map");
      ROS2SyncedRobotModel syncedRobot = new ROS2SyncedRobotModel(new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS), ros2Node);

      RosPointCloudPublisher publisher = new RosPointCloudPublisher(PointType.XYZI, false);
      ros1Node.attachPublisher("/os_cloud_node2/points", publisher);

      GPUPlanarRegionUpdater gpuPlanarRegionUpdater = new GPUPlanarRegionUpdater();
      AtomicReference<RawGPUPlanarRegionList> regionsReference = new AtomicReference<>();
      AbstractRosTopicSubscriber<RawGPUPlanarRegionList> mapsenseSubscriber = new AbstractRosTopicSubscriber<RawGPUPlanarRegionList>(RawGPUPlanarRegionList._TYPE)
      {
         @Override
         public void onNewMessage(RawGPUPlanarRegionList rawGPUPlanarRegionList)
         {
            regionsReference.set(rawGPUPlanarRegionList);
         }
      };
      ros1Node.attachSubscriber(RosTools.MAPSENSE_REGIONS, mapsenseSubscriber);

      RigidBodyTransform transformChestToL515DepthCamera = new RigidBodyTransform();
      transformChestToL515DepthCamera.setIdentity();
      transformChestToL515DepthCamera.getTranslation().set(0.275000, 0.052000, 0.140000);
      transformChestToL515DepthCamera.getRotation().setYawPitchRoll(0.010000, 1.151900, 0.045000);
      ReferenceFrame steppingFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("steppingCamera", syncedRobot.getReferenceFrames().getChestFrame(), transformChestToL515DepthCamera);

      RigidBodyTransform zForwardXRightToZUpXForward = new RigidBodyTransform();
      zForwardXRightToZUpXForward.appendPitchRotation(Math.PI / 2.0);
      zForwardXRightToZUpXForward.appendYawRotation(-Math.PI / 2.0);

      ROS2PublisherBasics<PlanarRegionsListMessage> regionsPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                                                        PlanarRegionsListMessage.class,
                                                                                                        REACommunicationProperties.outputTopic);

      RosTfPublisher tfPublisher = new RosTfPublisher(ros1Node, null);

      new Thread(() ->
                 {
                    while (!ros1Node.isStarted())
                    {
                       LogTools.info("waiting to connect...");
                       ThreadTools.sleep(500);
                    }

                    while (true)
                    {
                       PointCloud2 pointCloud = inputPointCloud.getAndSet(null);
                       RawGPUPlanarRegionList regions = regionsReference.getAndSet(null);

                       if (pointCloud != null || regions != null)
                       {
                          syncedRobot.update();
                       }

                       if (pointCloud != null)
                       {
                          ReferenceFrame ousterFrame = syncedRobot.getReferenceFrames().getOusterLidarFrame();
                          RigidBodyTransform ousterToWorld = ousterFrame.getTransformToWorldFrame();

                          if (debug)
                             System.out.println(ousterToWorld);

                          Time currentTime = ros1Node.getCurrentTime();
                          long timestamp = currentTime.totalNsecs();
                          LogTools.info("Broadcasting height map message");
                          tfPublisher.publish(ousterToWorld, timestamp, "odom", "os_sensor");

                          pointCloud.getHeader().setStamp(currentTime);
                          publisher.publish(pointCloud);
                       }

                       if (regions != null)
                       {
                          PlanarRegionsList planarRegionsList = gpuPlanarRegionUpdater.generatePlanarRegions(regions);
                          planarRegionsList.applyTransform(zForwardXRightToZUpXForward);
                          planarRegionsList.applyTransform(steppingFrame.getTransformToWorldFrame());

                          PlanarRegionsListMessage planarRegionsListMessage =  PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
                          regionsPublisher.publish(planarRegionsListMessage);
                          LogTools.info("Broadcasting mapsense regions");
                       }

                       ThreadTools.sleep(350);
                    }
                 }).start();
   }

   private static PointCloud2 packMessage(PointCloud2 message, Point3D[] points)
   {
      message.getHeader().setFrameId("base_footprint");
      PointType pointType = PointType.XYZI;
      message.setHeight(1);
      message.setWidth(points.length);
      message.setPointStep(pointType.getPointStep());
      int dataLength = pointType.getPointStep() * points.length;
      message.setRowStep(dataLength);
      message.setIsBigendian(false);
      message.setIsDense(true);
      message.setFields(pointType.getPointField());

      ChannelBuffer buffer = new LittleEndianHeapChannelBuffer(dataLength);
      for (int i = 0; i < points.length; i++)
      {
         buffer.writeFloat((float) points[i].getX());
         buffer.writeFloat((float) points[i].getY());
         buffer.writeFloat((float) points[i].getZ());
         buffer.writeFloat(100.0f);
      }
      message.setData(buffer);

      return message;
   }

   public static void main(String[] args) throws URISyntaxException
   {
      new AtlasHeightMapTopicConverter();
   }
}
