package us.ihmc.robotEnvironmentAwareness.hardware;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.application.Application;
import javafx.stage.DirectoryChooser;
import javafx.stage.Stage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.io.StereoVisionPointCloudDataExporter;
import us.ihmc.ros2.Ros2Node;

public class SimulatedStereoVisionPointCloudPublisher extends Application
{
   private static long DEFAULT_PUBLISHING_PERIOD_MS = 500;

   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "stereoVisionPublisherNode");
   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> stereoVisionPublisher = ROS2Tools.createPublisher(ros2Node,
                                                                                                                    StereoVisionPointCloudMessage.class,
                                                                                                                    ROS2Tools.getDefaultTopicNameGenerator());

   private final List<StereoVisionPointCloudMessage> listOfMessages = new ArrayList<>();

   private int indexToPublish = 0;
   private final List<Long> timestampsInTimeOrder = new ArrayList<>();
   private final Map<Long, StereoVisionPointCloudMessage> mapTimestampToStereoMessage = new HashMap<Long, StereoVisionPointCloudMessage>();

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1, getClass(), ExceptionHandling.CATCH_AND_REPORT);

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      System.out.println(this.getClass().getSimpleName());

      File initialDataFolder = new File("../");
      DirectoryChooser directoryChooser = new DirectoryChooser();
      directoryChooser.setInitialDirectory(initialDataFolder);
      File selectedDataFolder = directoryChooser.showDialog(primaryStage);
      File[] listOfFiles = selectedDataFolder.listFiles();

      List<File> sensorPoseFiles = new ArrayList<>();
      List<File> pointCloudFiles = new ArrayList<>();
      List<Long> timestamps = new ArrayList<Long>();

      for (File file : listOfFiles)
      {
         if (file.isFile())
         {
            String fileName = file.getName(); //TODO: sorting out the time stamp which is before the '.txt'.

            if (fileName.contains(StereoVisionPointCloudDataExporter.SENSOR_POSE_FILE_NAME_HEADER))
               sensorPoseFiles.add(file);

            if (fileName.contains(StereoVisionPointCloudDataExporter.POINT_CLOUD_FILE_NAME_HEADER))
               pointCloudFiles.add(file);
         }
      }

      for (int i = 0; i < sensorPoseFiles.size(); i++)
      {
         File sensorPoseFile = sensorPoseFiles.get(i);
         long sensorPoseTimestamp = extractTimestamp(sensorPoseFile.getName());

         for (int j = 0; j < pointCloudFiles.size(); j++)
         {
            File pointCloudFile = pointCloudFiles.get(j);
            long pointCloudTimestamp = extractTimestamp(pointCloudFile.getName());
            if (sensorPoseTimestamp == pointCloudTimestamp)
            {
               timestamps.add(sensorPoseTimestamp);
               mapTimestampToStereoMessage.put(sensorPoseTimestamp, StereoVisionPointCloudDataLoader.getMessageFromFile(sensorPoseFile, pointCloudFile));
            }
         }
      }

      int numberOfMessages = timestamps.size();
      for (int i = 0; i < numberOfMessages; i++)
      {
         long minTimestamp = Long.MAX_VALUE;
         for (int j = 0; j < timestamps.size(); j++)
         {
            if (timestamps.get(j) < minTimestamp)
            {
               minTimestamp = timestamps.get(j);
            }
         }

         if (timestamps.remove(minTimestamp))
         {
            timestampsInTimeOrder.add(minTimestamp);
         }
      }

      System.out.println("Importing Data Is Done " + listOfMessages.size() + " messages.");
      System.out.println("Publishing Is Started.");

      executorService.scheduleAtFixedRate(this::publish, 0, DEFAULT_PUBLISHING_PERIOD_MS, TimeUnit.MILLISECONDS);

      System.out.println("Publishing Is Done.");
   }

   private void publish()
   {
      if (timestampsInTimeOrder.size() == indexToPublish)
      {
         try
         {
            executorService.shutdown();
            System.exit(0);
            return;
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      }
      else
      {
         System.out.println("publish " + indexToPublish);
      }
      StereoVisionPointCloudMessage stereoVisionPointCloudMessage = mapTimestampToStereoMessage.get(timestampsInTimeOrder.get(indexToPublish));
      stereoVisionPublisher.publish(stereoVisionPointCloudMessage);
      indexToPublish++;
   }

   public static void main(String[] args)
   {
      launch();
   }

   private static long extractTimestamp(String fileName)
   {
      String[] stringsWithoutSpliter = fileName.split(StereoVisionPointCloudDataExporter.STEREO_DATA_SPLITER);
      for (String string : stringsWithoutSpliter)
      {
         if (string.contains(StereoVisionPointCloudDataExporter.STEREO_DATA_EXTENSION))
         {
            String[] timestampWithDot = string.split(StereoVisionPointCloudDataExporter.STEREO_DATA_EXTENSION);
            long timestamp = Long.parseLong(timestampWithDot[0]);
            return timestamp;
         }
      }
      return -1;
   }
}
