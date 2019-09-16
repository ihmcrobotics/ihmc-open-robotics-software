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
}
