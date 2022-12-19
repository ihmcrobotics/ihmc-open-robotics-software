package us.ihmc.avatar.heightMap;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.common.util.concurrent.AtomicDouble;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.HeightMapMessagePubSubType;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapManager;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class HeightMapUpdater
{
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
   private final AbstractHeightMapUpdater heightMapUpdater;
   private final ExecutorService heightMapUpdaterService = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private final AtomicBoolean updateThreadIsRunning = new AtomicBoolean();

   public HeightMapUpdater(Messager messager, ROS2Node ros2Node, Stage stage)
   {
      this.stage = stage;
      this.messager = messager;

      IHMCROS2Publisher<HeightMapMessage> heightMapPublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.HEIGHT_MAP_OUTPUT);

      Consumer<HeightMapMessage> heightMapDataConsumer = (message) ->
      {
         messager.submitMessage(HeightMapMessagerAPI.HeightMapData, message);
         heightMapPublisher.publish(message);
      };

      heightMapUpdater = new AbstractHeightMapUpdater(heightMapDataConsumer);

      messager.registerTopicListener(HeightMapMessagerAPI.PointCloudData, pointCloudData ->
      {
         if (heightMapUpdater.updatesAreEnabled())
         {
            heightMapUpdater.addPointCloudToQueue(pointCloudData);

            if (!updateThreadIsRunning.getAndSet(true))
            {
               heightMapUpdaterService.execute(() ->
                                               {
                                                  updateThreadIsRunning.set(!heightMapUpdater.runUpdateThread());
                                               });
            }
         }
      });


      messager.registerTopicListener(HeightMapMessagerAPI.EnableUpdates, heightMapUpdater::setEnableUpdates);
      messager.registerTopicListener(HeightMapMessagerAPI.Clear, c -> heightMapUpdater.requestClear());
      messager.registerTopicListener(HeightMapMessagerAPI.GridCenterX, heightMapUpdater::setGridCenterX);
      messager.registerTopicListener(HeightMapMessagerAPI.GridCenterY, heightMapUpdater::setGridCenterY);
      messager.registerTopicListener(HeightMapMessagerAPI.MaxHeight, heightMapUpdater::setMaxHeight);

      messager.registerTopicListener(HeightMapMessagerAPI.PublishFrequency, heightMapUpdater::setPublishFrequency);
      messager.registerTopicListener(HeightMapMessagerAPI.Export, e -> heightMapUpdater.exportOnThread());
      messager.registerTopicListener(HeightMapMessagerAPI.Import, i -> importHeightMap());
   }


   private final AtomicBoolean importing = new AtomicBoolean();

   public void importHeightMap()
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
