package us.ihmc.ihmcPerception.bagTools;

import controller_msgs.msg.dds.LidarScanMessage;

import javax.swing.*;
import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

/**
 * Input: a file containing the output of "rostopic echo [point cloud topic]" for a single message instance
 * Output: converting that point cloud to a LidarScanMessage and writing to disk
 */
public class PointCloudMessageImporter
{
   public PointCloudMessageImporter() throws Exception
   {
      JFileChooser fileChooser = new JFileChooser();
      int chooserState = fileChooser.showOpenDialog(null);

      if (chooserState != JFileChooser.APPROVE_OPTION)
      {
         return;
      }

      BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(new FileInputStream(fileChooser.getSelectedFile())));

      int height = 1;
      int width = 40494;
      int pointStep = 32;
      LidarScanMessage lidarScanMessage = new LidarScanMessage();

      int numberOfPoints = height * width;

      while (true)
      {
         String line = bufferedReader.readLine();
         if (!line.startsWith("data"))
         {
            continue;
         }

         String[] csv = line.substring(7, line.length() - 1).split(", ");
         byte[] data = new byte[csv.length];
         for (int i = 0; i < csv.length; i++)
         {
            data[i] = (byte) Integer.parseInt(csv[i]);
         }

         float[] points = new float[3 * numberOfPoints];
         byte[] bytes = new byte[4];

         for (int i = 0; i < numberOfPoints; i++)
         {
            int startIndex = pointStep * i;
            packBytes(bytes, startIndex + 0, data);
            float x = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).getFloat();
            packBytes(bytes, startIndex + 4, data);
            float y = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).getFloat();
            packBytes(bytes, startIndex + 8, data);
            float z = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).getFloat();

            points[3 * i + 0] = x;
            points[3 * i + 1] = y;
            points[3 * i + 2] = z;
         }

         lidarScanMessage.getScan().add(points);
         break;
      }

      lidarScanMessage.getLidarPosition().set(0.0, 0.0, 1.5);
      List<LidarScanMessage> messageList = new ArrayList<>();
      messageList.add(lidarScanMessage);

      LidarScanMessageExporter.export(messageList);
   }

   static void packBytes(byte[] bytesToPack, int startIndex, byte[] data)
   {
      for (int i = 0; i < 4; i++)
      {
         bytesToPack[i] = data[startIndex + i];
      }
   }

   public static void main(String[] args) throws Exception
   {
      new PointCloudMessageImporter();
   }
}
