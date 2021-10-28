package us.ihmc.avatar.heightMap;

import controller_msgs.msg.dds.HeightMapMessage;
import org.apache.commons.lang3.tuple.Pair;
import sensor_msgs.PointCloud2;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapManager;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

public class HeightMapUpdater
{
   static final boolean USE_OUSTER_FRAME = false;
   static final RigidBodyTransform APPROX_OUSTER_TRANSFORM = new RigidBodyTransform();
   static
   {
//      double ousterPitch = Math.toRadians(21.5);
//      APPROX_OUSTER_TRANSFORM.getRotation().setToPitchOrientation(ousterPitch);

      double ousterPitch = Math.toRadians(30.0);
      APPROX_OUSTER_TRANSFORM.getRotation().setToPitchOrientation(ousterPitch);
      APPROX_OUSTER_TRANSFORM.getTranslation().set(-0.2, 0.0, 1.0);
   }

   private final Messager messager;
   private final HeightMapParameters parameters;
   private final ExecutorService heightMapUpdater = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private final AtomicBoolean processing = new AtomicBoolean();

   private final PoseReferenceFrame ousterFrame = new PoseReferenceFrame("ousterFrame", ReferenceFrame.getWorldFrame());
   private final HeightMapManager heightMap;
   private final IHMCROS2Publisher<HeightMapMessage> publisher;

   private static final double FLYING_POINT_MIN_X = 1.092;
   private static final double FLYING_POINT_MAX_X = 1.3;
   private static final double FLYING_POINT_MIN_Y = -0.2;
   private static final double FLYING_POINT_MAX_Y = 0.1;
   private static final BoundingBox3D FLYING_POINT_BOUNDING_BOX = new BoundingBox3D();
   private static final long[] FLYING_POINT_COUNTERS = new long[10];
   private static long updateCount = 0;

   static
   {
      FLYING_POINT_BOUNDING_BOX.set(FLYING_POINT_MIN_X, FLYING_POINT_MIN_Y, -10.0, FLYING_POINT_MAX_X, FLYING_POINT_MAX_Y, 10.0);
   }

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
   }

   private void update(Pair<PointCloud2, FramePose3D> pointCloudData)
   {
      PointCloudData pointCloud = new PointCloudData(pointCloudData.getKey(), 1000000, false);
      ousterFrame.setPoseAndUpdate(pointCloudData.getRight());

      if (USE_OUSTER_FRAME)
      {
         // Transform ouster data
         for (int i = 0; i < pointCloud.getPointCloud().length; i++)
         {
            FramePoint3D point = new FramePoint3D(ousterFrame, pointCloud.getPointCloud()[i]);
            point.changeFrame(ReferenceFrame.getWorldFrame());
            pointCloud.getPointCloud()[i].set(point);
         }
      }
      else
      {
//         int flyingPointDebugCount = 0;

         for (int i = 0; i < pointCloud.getPointCloud().length; i++)
         {
            pointCloud.getPointCloud()[i].applyTransform(APPROX_OUSTER_TRANSFORM);
//            if (FLYING_POINT_BOUNDING_BOX.isInsideInclusive(pointCloud.getPointCloud()[i]))
//               flyingPointDebugCount++;
         }

//         FLYING_POINT_COUNTERS[flyingPointDebugCount] = FLYING_POINT_COUNTERS[flyingPointDebugCount] + 1;
//         if (updateCount++ > 500)
//         {
//            for (int i = 0; i < FLYING_POINT_COUNTERS.length; i++)
//            {
//               System.out.println(i + "\t" + FLYING_POINT_COUNTERS[i]);
//            }
//            updateCount = 0;
//         }
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
