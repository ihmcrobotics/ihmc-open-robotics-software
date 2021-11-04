package us.ihmc.avatar.heightMap;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import controller_msgs.msg.dds.HeightMapMessage;
import controller_msgs.msg.dds.HeightMapMessagePubSubType;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import org.apache.commons.lang3.tuple.Pair;
import sensor_msgs.PointCloud2;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapManager;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

public class HeightMapUpdater
{
   private static final boolean printFrequency = false;

   static final boolean USE_OUSTER_FRAME = true;
   static final RigidBodyTransform APPROX_OUSTER_TRANSFORM = new RigidBodyTransform();
   static
   {
//      double ousterPitch = Math.toRadians(21.5);
//      APPROX_OUSTER_TRANSFORM.getRotation().setToPitchOrientation(ousterPitch);

      double ousterPitch = Math.toRadians(30.0);
      APPROX_OUSTER_TRANSFORM.getRotation().setToPitchOrientation(ousterPitch);
      APPROX_OUSTER_TRANSFORM.getTranslation().set(-0.2, 0.0, 1.0);
   }

   private final Stage stage;
   private final Messager messager;
   private final HeightMapParameters parameters;
   private final ExecutorService heightMapUpdater = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private final AtomicBoolean heightMapLock = new AtomicBoolean();

   private final PoseReferenceFrame ousterFrame = new PoseReferenceFrame("ousterFrame", ReferenceFrame.getWorldFrame());
   private final HeightMapManager heightMap;
   private final IHMCROS2Publisher<HeightMapMessage> publisher;
   private final AtomicReference<Point2D> gridCenter;
   private final AtomicReference<HeightMapMessage> latestMessage = new AtomicReference<>();

   private int publishFrequencyCounter = 0;
   private final AtomicInteger publishFrequency = new AtomicInteger();

   private static final double FLYING_POINT_MIN_X = 1.092;
   private static final double FLYING_POINT_MAX_X = 1.3;
   private static final double FLYING_POINT_MIN_Y = -0.2;
   private static final double FLYING_POINT_MAX_Y = 0.1;
   private static final BoundingBox3D FLYING_POINT_BOUNDING_BOX = new BoundingBox3D();
   private static final long[] FLYING_POINT_COUNTERS = new long[10];

   static
   {
      FLYING_POINT_BOUNDING_BOX.set(FLYING_POINT_MIN_X, FLYING_POINT_MIN_Y, -10.0, FLYING_POINT_MAX_X, FLYING_POINT_MAX_Y, 10.0);
   }

   public HeightMapUpdater(Messager messager, ROS2Node ros2Node, Stage stage)
   {
      this.stage = stage;
      this.messager = messager;
      publisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.HEIGHT_MAP_OUTPUT);

      parameters = new HeightMapParameters();
      heightMap = new HeightMapManager(parameters.getGridResolutionXY(), parameters.getGridSizeXY());
      AtomicReference<Boolean> enableUpdates = messager.createInput(HeightMapMessagerAPI.EnableUpdates);

      messager.registerTopicListener(HeightMapMessagerAPI.PointCloudData, pointCloudData ->
      {
         if (!enableUpdates.get())
         {
            return;
         }

         if (!heightMapLock.getAndSet(true))
         {
            heightMapUpdater.execute(() -> update(pointCloudData));
         }
      });

      gridCenter = messager.createInput(HeightMapMessagerAPI.GridCenter);
      messager.registerTopicListener(HeightMapMessagerAPI.PublishFrequency, publishFrequency::set);
      messager.registerTopicListener(HeightMapMessagerAPI.Export, e -> ThreadTools.startAThread(this::export, "Height map exporter"));
      messager.registerTopicListener(HeightMapMessagerAPI.Import, i -> importHeightMap());
   }

   private void update(Pair<PointCloud2, FramePose3D> pointCloudData)
   {
      if (printFrequency)
      {
         updateFrequency();
      }

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
         for (int i = 0; i < pointCloud.getPointCloud().length; i++)
         {
            pointCloud.getPointCloud()[i].applyTransform(APPROX_OUSTER_TRANSFORM);
         }
      }

      Point2D gridCenter = this.gridCenter.getAndSet(null);
      if (gridCenter != null)
      {
         heightMap.setGridCenter(gridCenter.getX(), gridCenter.getY());
      }

      // Update height map
      heightMap.update(pointCloud.getPointCloud());

      if (--publishFrequencyCounter <= 0)
      {
         HeightMapMessage message = buildMessage();
         messager.submitMessage(HeightMapMessagerAPI.HeightMapData, message);
         publisher.publish(message);

         publishFrequencyCounter = publishFrequency.get();
         latestMessage.set(message);
      }

      heightMapLock.set(false);
   }

   private HeightMapMessage buildMessage()
   {
      // Copy and report over messager
      HeightMapMessage message = new HeightMapMessage();
      message.setGridSizeXy(parameters.getGridSizeXY());
      message.setXyResolution(parameters.getGridResolutionXY());
      message.setGridCenterX(heightMap.getGridCenterXY().getX());
      message.setGridCenterY(heightMap.getGridCenterXY().getY());

      message.getXCells().addAll(heightMap.getXCells());
      message.getYCells().addAll(heightMap.getYCells());
      for (int i = 0; i < heightMap.getXCells().size(); i++)
      {
         message.getHeights().add((float) heightMap.getHeightAt(heightMap.getXCells().get(i), heightMap.getYCells().get(i)));
      }

      return message;
   }

   private boolean firstTick = true;
   private long startTime;
   private int updateFrequencyCounter = 0;

   private void updateFrequency()
   {
      if (firstTick)
      {
         startTime = System.currentTimeMillis();
         firstTick = false;
      }
      else
      {
         updateFrequencyCounter++;
         double timePerCallSeconds = 1e-3 * ((double) (System.currentTimeMillis() - startTime) / updateFrequencyCounter);
         System.out.println("Frequency: " + (1 / timePerCallSeconds) + " Hz");
      }
   }

   private void export()
   {
      HeightMapMessage message = latestMessage.get();
      if (message == null)
      {
         LogTools.info("No height map available, cannot export.");
         return;
      }

      try
      {
         JSONSerializer<HeightMapMessage> serializer = new JSONSerializer<>(new HeightMapMessagePubSubType());
         byte[] serializedHeightMap = serializer.serializeToBytes(message);

         SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
         String fileName = "HeightMap" + dateFormat.format(new Date()) + ".json";
         String file = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator + fileName;

         FileTools.ensureFileExists(new File(file).toPath());
         FileOutputStream outputStream = new FileOutputStream(file);
         PrintStream printStream = new PrintStream(outputStream);

         printStream.write(serializedHeightMap);
         printStream.flush();
         outputStream.close();
         printStream.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private final AtomicBoolean importing = new AtomicBoolean();

   private void importHeightMap()
   {
      ThreadTools.startAThread(() ->
                               {
                                  if (importing.getAndSet(true))
                                  {
                                     return;
                                  }

                                  messager.submitMessage(HeightMapMessagerAPI.EnableUpdates, false);

                                  FileChooser fileChooser = new FileChooser();
                                  fileChooser.setTitle("Import Height Map");
                                  fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Height Map Files (*.json)", "*.json"));
                                  File file = fileChooser.showOpenDialog(stage);

                                  ObjectMapper objectMapper = new ObjectMapper();
                                  try
                                  {
                                     JSONSerializer<HeightMapMessage> serializer = new JSONSerializer<>(new HeightMapMessagePubSubType());
                                     InputStream inputStream = new FileInputStream(file);
                                     JsonNode jsonNode = objectMapper.readTree(inputStream);
                                     HeightMapMessage heightMapMessage = serializer.deserialize(jsonNode.toString());
                                     inputStream.close();

                                     messager.submitMessage(HeightMapMessagerAPI.HeightMapData, heightMapMessage);
                                  }
                                  catch (IOException e)
                                  {
                                     e.printStackTrace();
                                  }

                                  importing.set(false);
                               }, "Height map importer");
   }
}
