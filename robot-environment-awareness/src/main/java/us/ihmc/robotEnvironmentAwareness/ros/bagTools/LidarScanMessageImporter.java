package us.ihmc.robotEnvironmentAwareness.ros.bagTools;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.LidarScanMessagePubSubType;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;

import javax.swing.*;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

/**
 * Loads and broadcasts to JSON file written by LidarScanMessageExporter, analogous to "rosbag play"
 */
public class LidarScanMessageImporter
{
   private static final long WAYMO_PUBLISH_PERIOD = 100; // 10Hz update rate
   private static final long CATPACK_PUBLISH_PERIOD = 4; // 100Hz update rate

   private static final long PUBLISH_PERIOD = CATPACK_PUBLISH_PERIOD;

   // useful if there's just a single message
   private static final int PUBLISH_N_TIMES = 10;

   public LidarScanMessageImporter(InputStream inputStream) throws IOException
   {
      ObjectMapper objectMapper = new ObjectMapper();
      JsonNode jsonNode = objectMapper.readTree(inputStream);
      int numberOfMessages = jsonNode.size();

      List<LidarScanMessage> lidarScanMessages = new ArrayList<>();
      JSONSerializer<LidarScanMessage> messageSerializer = new JSONSerializer<>(new LidarScanMessagePubSubType());

      for (int i = 0; i < numberOfMessages; i++)
      {
         lidarScanMessages.add(messageSerializer.deserialize(jsonNode.get(i).toString()));
      }

      System.out.println("Loaded " + numberOfMessages + " messages");

      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, getClass().getSimpleName());
      IHMCROS2Publisher<LidarScanMessage> publisher = ROS2Tools.createPublisher(ros2Node, LidarScanMessage.class, "/ihmc/lidar_scan");

      for (int i = 0; i < PUBLISH_N_TIMES; i++)
      {
         for (int j = 0; j < lidarScanMessages.size(); j++)
         {
            publisher.publish(lidarScanMessages.get(j));
            ThreadTools.sleep(PUBLISH_PERIOD);
         }

         ThreadTools.sleep(1000);
      }

      System.out.println("Finished publishing");
   }

   public static void main(String[] args) throws IOException
   {
      JFileChooser outputChooser = new JFileChooser();
      int returnVal = outputChooser.showSaveDialog(null);
      if (returnVal != JFileChooser.APPROVE_OPTION)
      {
         return;
      }

      File file = outputChooser.getSelectedFile();
      InputStream inputStream = new FileInputStream(file);
      new LidarScanMessageImporter(inputStream);
   }
}
