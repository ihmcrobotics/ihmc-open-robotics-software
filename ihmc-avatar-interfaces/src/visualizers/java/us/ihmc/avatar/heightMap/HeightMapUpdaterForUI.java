package us.ihmc.avatar.heightMap;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.HeightMapMessagePubSubType;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.perception.heightMap.HeightMapUpdater;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.ROS2NodeInterface;

import java.io.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

public class HeightMapUpdaterForUI
{
   private final Stage stage;
   private final Messager messager;
   private final HeightMapUpdater heightMapUpdater;
   private final ExecutorService heightMapUpdaterService = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private final AtomicBoolean updateThreadIsRunning = new AtomicBoolean();

   public HeightMapUpdaterForUI(Messager messager, ROS2NodeInterface ros2Node, Stage stage)
   {
      this.stage = stage;
      this.messager = messager;

      heightMapUpdater = new HeightMapUpdater();

      messager.addTopicListener(HeightMapMessagerAPI.PointCloudData, pointCloudData ->
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


      attachMessagerToUpdater();

      ROS2PublisherBasics<HeightMapMessage> heightMapPublisher = ros2Node.createPublisher(PerceptionAPI.HEIGHT_MAP_OUTPUT);
      heightMapUpdater.attachHeightMapConsumer(heightMap -> messager.submitMessage(HeightMapMessagerAPI.HeightMapData, heightMap));
      heightMapUpdater.attachHeightMapConsumer(heightMapPublisher::publish);

      messager.addTopicListener(HeightMapMessagerAPI.Import, i -> importHeightMap());
   }

   private void attachMessagerToUpdater()
   {
      messager.addTopicListener(HeightMapMessagerAPI.EnableUpdates, heightMapUpdater::setEnableUpdates);
      messager.addTopicListener(HeightMapMessagerAPI.Clear, c -> heightMapUpdater.requestClear());
      messager.addTopicListener(HeightMapMessagerAPI.GridCenterX, heightMapUpdater::setGridCenterX);
      messager.addTopicListener(HeightMapMessagerAPI.GridCenterY, heightMapUpdater::setGridCenterY);
      messager.addTopicListener(HeightMapMessagerAPI.MaxHeight, heightMapUpdater::setMaxHeight);
      messager.addTopicListener(HeightMapMessagerAPI.parameters, heightMapUpdater::setParameters);

      messager.addTopicListener(HeightMapMessagerAPI.PublishFrequency, heightMapUpdater::setPublishFrequency);
      messager.addTopicListener(HeightMapMessagerAPI.Export, e -> heightMapUpdater.exportOnThread());

      heightMapUpdater.setGridCenterConsumer((point) ->
                                             {
                                                messager.submitMessage(HeightMapMessagerAPI.GridCenterX, point.getX());
                                                messager.submitMessage(HeightMapMessagerAPI.GridCenterY, point.getY());
                                             });
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
