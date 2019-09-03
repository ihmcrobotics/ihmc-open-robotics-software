package us.ihmc.robotEnvironmentAwareness.hardware;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
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

      for (File file : listOfFiles)
      {
         if (file.isFile())
         {
            String fileName = file.getName(); //TODO: sorting out the time stamp which is before the '.txt'.
            listOfMessages.add(StereoVisionPointCloudDataLoader.getMessageFromFile(file));
         }
      }
      System.out.println("Importing Data Is Done " + listOfMessages.size() + " messages.");
      System.out.println("Publishing Is Started.");

      executorService.scheduleAtFixedRate(this::publish, 0, DEFAULT_PUBLISHING_PERIOD_MS, TimeUnit.MILLISECONDS);

      System.out.println("Publishing Is Done.");
   }

   private void publish()
   {
      if (indexToPublish == listOfMessages.size())
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
         System.out.println("publish "+indexToPublish);
      }
      StereoVisionPointCloudMessage stereoVisionPointCloudMessage = listOfMessages.get(indexToPublish);
      stereoVisionPublisher.publish(stereoVisionPointCloudMessage);
      indexToPublish++;
   }

   public static void main(String[] args)
   {
      launch();
   }
}
