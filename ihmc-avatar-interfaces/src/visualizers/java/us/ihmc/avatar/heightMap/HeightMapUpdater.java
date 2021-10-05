package us.ihmc.avatar.heightMap;

import controller_msgs.msg.dds.HeightMapMessage;
import org.apache.commons.lang3.tuple.Pair;
import sensor_msgs.PointCloud2;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapManager;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

public class HeightMapUpdater
{
   private final Messager messager;
   private final HeightMapParameters parameters;
   private final ExecutorService heightMapUpdater = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private final AtomicBoolean processing = new AtomicBoolean();

   private final PoseReferenceFrame ousterFrame = new PoseReferenceFrame("ousterFrame", ReferenceFrame.getWorldFrame());
   private final HeightMapManager heightMap;
   private final IHMCROS2Publisher<HeightMapMessage> publisher;

   public HeightMapUpdater(Messager messager, ROS2Node ros2Node)
   {
      this.messager = messager;
      publisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.HEIGHT_MAP_OUTPUT);

      parameters = new HeightMapParameters();
      heightMap = new HeightMapManager(parameters.getGridResolutionXY(), parameters.getGridSizeXY());

      messager.registerTopicListener(HeightMapMessagerAPI.PointCloudData, pointCloudData ->
      {
         if (!processing.getAndSet(true))
         {
            heightMapUpdater.execute(() -> update(pointCloudData));
         }
      });

//      transform.getRotation().setToPitchOrientation(Math.toRadians(30.0));
//      transform.getTranslation().set(-0.2, 0.0, 1.0);
   }

   private void update(Pair<PointCloud2, FramePose3D> pointCloudData)
   {
      PointCloudData pointCloud = new PointCloudData(pointCloudData.getKey(), 1000000, false);
      ousterFrame.setPoseAndUpdate(pointCloudData.getRight());

      // Transform ouster data
      for (int i = 0; i < pointCloud.getPointCloud().length; i++)
      {
         FramePoint3D point = new FramePoint3D(ousterFrame, pointCloud.getPointCloud()[i]);
         point.changeFrame(ReferenceFrame.getWorldFrame());
         pointCloud.getPointCloud()[i].set(point);
      }

      // Update height map
      heightMap.update(pointCloud.getPointCloud());

      // Copy and report over messager
      HeightMapMessage message = new HeightMapMessage();
      message.setGridSizeXy(parameters.getGridSizeXY());
      message.setXyResolution(parameters.getGridResolutionXY());

      message.getXCells().addAll(heightMap.getXCells());
      message.getYCells().addAll(heightMap.getYCells());
      for (int i = 0; i < heightMap.getXCells().size(); i++)
      {
         message.getHeights().add((float) heightMap.getHeightAt(heightMap.getXCells().get(i), heightMap.getYCells().get(i)));
      }

      messager.submitMessage(HeightMapMessagerAPI.HeightMapData, message);
      publisher.publish(message);

      processing.set(false);
   }
}
