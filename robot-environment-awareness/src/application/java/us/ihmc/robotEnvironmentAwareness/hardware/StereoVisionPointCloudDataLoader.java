package us.ihmc.robotEnvironmentAwareness.hardware;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.packets.MessageTools;

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
}
