package us.ihmc.ihmcPerception.bagTools;

import controller_msgs.msg.dds.LidarScanMessage;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Point3D;

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
   /** PointCloud2 PointField data */
   private static final ByteOrder byteOrder = ByteOrder.LITTLE_ENDIAN;
   private static final int height = 1;
   private static final int pointStep = 20;

   private static final Point3D sensorPosition = new Point3D(0.0, 0.0, 0.0);

   public PointCloudMessageImporter() throws Exception
   {
      JFileChooser fileChooser = new JFileChooser();
      int chooserState = fileChooser.showOpenDialog(null);

      AxisAngle transform = new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(-135.0));

      if (chooserState != JFileChooser.APPROVE_OPTION)
      {
         return;
      }

      BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(new FileInputStream(fileChooser.getSelectedFile())));

      LidarScanMessage lidarScanMessage = new LidarScanMessage();

      while (true)
      {
         String line = bufferedReader.readLine();

         if (line == null)
         {
            break;
         }
         else if (!line.startsWith("data"))
         {
            continue;
         }

         int initialCharacter = 7; // length of "data: ["
         int finalCharacter = line.length() - 1; // ignore trailing bracket

         String[] csv = line.substring(initialCharacter, finalCharacter).split(", ");
         byte[] data = new byte[csv.length];
         for (int i = 0; i < csv.length; i++)
         {
            data[i] = (byte) Integer.parseInt(csv[i]);
         }

         int numberOfPoints = csv.length / pointStep;
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

            Point3D point = new Point3D(x, y, z);
            transform.transform(point);

            points[3 * i + 0] = (float) point.getX();
            points[3 * i + 1] = (float) point.getY();
            points[3 * i + 2] = (float) point.getZ();
         }

         lidarScanMessage.getScan().add(points);
         break;
      }

      lidarScanMessage.getLidarPosition().set(sensorPosition);
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
