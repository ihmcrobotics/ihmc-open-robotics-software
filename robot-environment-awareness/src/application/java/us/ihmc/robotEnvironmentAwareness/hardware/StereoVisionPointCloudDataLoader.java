package us.ihmc.robotEnvironmentAwareness.hardware;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.robotEnvironmentAwareness.ui.io.StereoVisionPointCloudDataExporter;

public class StereoVisionPointCloudDataLoader
{
   public static StereoVisionPointCloudMessage getMessageFromFile(File dataFile)
   {
      if (!dataFile.canRead())
         new NullPointerException("No dataFile");

      int maximumNumberOfPoints = 200000;
      double[] pointCloudBuffer = new double[maximumNumberOfPoints * 3];
      int[] colorBuffer = new int[maximumNumberOfPoints];
      BufferedReader bufferedReader = null;
      try
      {
         bufferedReader = new BufferedReader(new FileReader(dataFile));
      }
      catch (FileNotFoundException e1)
      {
         e1.printStackTrace();
      }

      int lineIndex = 0;
      while (true)
      {
         String lineJustFetched = null;
         String[] idxyzcolorArray;
         try
         {
            lineJustFetched = bufferedReader.readLine();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         if (lineJustFetched == null)
         {
            break;
         }
         else
         {
            idxyzcolorArray = lineJustFetched.split("\t");
            Integer.parseInt(idxyzcolorArray[0]);
            pointCloudBuffer[3 * lineIndex + 0] = Double.parseDouble(idxyzcolorArray[1]);
            pointCloudBuffer[3 * lineIndex + 1] = Double.parseDouble(idxyzcolorArray[2]);
            pointCloudBuffer[3 * lineIndex + 2] = Double.parseDouble(idxyzcolorArray[3]);
            colorBuffer[lineIndex] = Integer.parseInt(idxyzcolorArray[4]);

            lineIndex++;
         }
      }

      float[] resizedPointCloudBuffer = new float[lineIndex * 3];
      int[] resizedColorBuffer = new int[lineIndex];
      for (int i = 0; i < resizedPointCloudBuffer.length; i++)
         resizedPointCloudBuffer[i] = (float) pointCloudBuffer[i];
      for (int i = 0; i < resizedColorBuffer.length; i++)
         resizedColorBuffer[i] = colorBuffer[i];

      StereoVisionPointCloudMessage message = MessageTools.createStereoVisionPointCloudMessage(System.nanoTime(), resizedPointCloudBuffer, resizedColorBuffer);
      return message;
   }

   public static StereoVisionPointCloudMessage getMessageFromFile(File sensorPoseFile, File pointCloudFile)
   {
      if (!sensorPoseFile.canRead() || !pointCloudFile.canRead())
         new NullPointerException("No dataFile");

      BufferedReader bufferedReader = null;
      try
      {
         bufferedReader = new BufferedReader(new FileReader(sensorPoseFile));
      }
      catch (FileNotFoundException e1)
      {
         e1.printStackTrace();
      }

      String lineJustFetched = null;
      String[] positionQuaternianArray;
      try
      {
         lineJustFetched = bufferedReader.readLine();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      if (lineJustFetched == null)
      {
         return null;
      }
      else
      {
         positionQuaternianArray = lineJustFetched.split("\t");
         double px = Double.parseDouble(positionQuaternianArray[0]);
         double py = Double.parseDouble(positionQuaternianArray[1]);
         double pz = Double.parseDouble(positionQuaternianArray[2]);

         double ox = Double.parseDouble(positionQuaternianArray[3]);
         double oy = Double.parseDouble(positionQuaternianArray[4]);
         double oz = Double.parseDouble(positionQuaternianArray[5]);
         double os = Double.parseDouble(positionQuaternianArray[6]);

         StereoVisionPointCloudMessage message = getMessageFromFile(pointCloudFile);
         message.getSensorPosition().set(px, py, pz);
         message.getSensorOrientation().set(ox, oy, oz, os);
         return message;
      }
   }

   public static List<StereoVisionPointCloudMessage> getMessagesFromFile(File selectedDataFolder)
   {
      File[] listOfFiles = selectedDataFolder.listFiles();

      List<File> sensorPoseFiles = new ArrayList<>();
      List<File> pointCloudFiles = new ArrayList<>();
      List<Long> timestamps = new ArrayList<Long>();

      Map<Long, StereoVisionPointCloudMessage> mapTimestampToStereoMessage = new HashMap<Long, StereoVisionPointCloudMessage>();
      List<Long> timestampsInTimeOrder = new ArrayList<>();
      List<StereoVisionPointCloudMessage> messagesInTimeOrder = new ArrayList<>();

      for (File file : listOfFiles)
      {
         if (file.isFile())
         {
            String fileName = file.getName();

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

      for (int i = 0; i < numberOfMessages; i++)
         messagesInTimeOrder.add(mapTimestampToStereoMessage.get(timestampsInTimeOrder.get(i)));

      return messagesInTimeOrder;
   }

   public static long extractTimestamp(String fileName)
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
