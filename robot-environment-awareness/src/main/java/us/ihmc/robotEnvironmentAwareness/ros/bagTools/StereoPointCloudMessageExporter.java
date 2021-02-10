package us.ihmc.robotEnvironmentAwareness.ros.bagTools;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.LidarScanMessagePubSubType;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessagePubSubType;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.idl.serializers.extra.JSONSerializer;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;

/**
 * Writes a list of LidarScanMessage's to file, analogous to saving a "rosbag record"
 */
public class StereoPointCloudMessageExporter
{
   private static final DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss");

   static void export(List<StereoVisionPointCloudMessage> messageList)
   {
      try
      {
         String date = LocalDateTime.now().format(dateTimeFormatter);
         String file = System.getProperty("user.home") + File.separator + "LidarScanMessageList_" + date + ".json";
         JSONSerializer<StereoVisionPointCloudMessage> serializer = new JSONSerializer<>(new StereoVisionPointCloudMessagePubSubType());

         FileTools.ensureFileExists(new File(file).toPath());
         FileOutputStream outputStream = new FileOutputStream(file);
         PrintStream printStream = new PrintStream(outputStream);

         printStream.println("[");
         for (int i = 0; i < messageList.size(); i++)
         {
            byte[] serializedStereoScanMessage = serializer.serializeToBytes(messageList.get(i));
            printStream.write(serializedStereoScanMessage);

            if (i != messageList.size() - 1)
            {
               printStream.println(",");
            }
         }
         printStream.println("]");

         printStream.flush();
         outputStream.close();
         printStream.close();
         System.exit(0);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         System.exit(1);
      }
   }
}
