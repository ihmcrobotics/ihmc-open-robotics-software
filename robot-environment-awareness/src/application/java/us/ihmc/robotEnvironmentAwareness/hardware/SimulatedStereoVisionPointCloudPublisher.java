package us.ihmc.robotEnvironmentAwareness.hardware;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.stage.DirectoryChooser;
import javafx.stage.Stage;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.ui.io.StereoVisionPointCloudDataLoader;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.tools.thread.ExecutorServiceTools.ExceptionHandling;

public class SimulatedStereoVisionPointCloudPublisher extends ApplicationNoModule
{
   private static long DEFAULT_PUBLISHING_PERIOD_MS = 250;

   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "stereoVisionPublisherNode");
   private final ROS2PublisherBasics<StereoVisionPointCloudMessage> stereoVisionPublisher = ros2Node.createPublisher(PerceptionAPI.D435_POINT_CLOUD);

   private int indexToPublish = 0;
   private final List<StereoVisionPointCloudMessage> stereoVisionPointCloudMessagesToPublish = new ArrayList<>();

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1, getClass(), ExceptionHandling.CATCH_AND_REPORT);

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      System.out.println(this.getClass().getSimpleName());

      File initialDataFolder = new File("../");
      DirectoryChooser directoryChooser = new DirectoryChooser();
      directoryChooser.setInitialDirectory(initialDataFolder);
      File selectedDataFolder = directoryChooser.showDialog(primaryStage);

      stereoVisionPointCloudMessagesToPublish.addAll(StereoVisionPointCloudDataLoader.getMessagesFromFile(selectedDataFolder));

      System.out.println("Importing Data Is Done " + stereoVisionPointCloudMessagesToPublish.size() + " messages.");
      System.out.println("Publishing Is Started.");

      executorService.scheduleAtFixedRate(this::publish, 0, DEFAULT_PUBLISHING_PERIOD_MS, TimeUnit.MILLISECONDS);
   }

   private void publish()
   {
      if (stereoVisionPointCloudMessagesToPublish.size() == indexToPublish)
      {
         try
         {
            System.out.println("Publishing Is Done.");
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
      StereoVisionPointCloudMessage stereoVisionPointCloudMessage = stereoVisionPointCloudMessagesToPublish.get(indexToPublish);
      stereoVisionPublisher.publish(stereoVisionPointCloudMessage);
      indexToPublish++;
   }

   public static void main(String[] args)
   {
      launch();
   }

   public Map<Long, StereoVisionPointCloudMessage> getStereoVisionPointCloudMessages(File selectedDataFolder)
   {
      Map<Long, StereoVisionPointCloudMessage> mapTimestampToStereoMessage = new HashMap<Long, StereoVisionPointCloudMessage>();

      return mapTimestampToStereoMessage;
   }

}
