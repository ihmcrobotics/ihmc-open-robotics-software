package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import java.io.IOException;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Date;

import javax.vecmath.Point3d;

import georegression.struct.point.Point3D_F64;
import us.ihmc.darpaRoboticsChallenge.odometry.CarLocalizerFileWritingUtils;
import us.ihmc.tools.thread.ThreadTools;

public class LidarDataRecorder
{
   public static final String FILE_EXTENSION = ".dat";
   private static final int XYZ_VECTOR_LENGTH = 3;
   private static final String[] fileNames;

   static
   {
      fileNames = CarLocalizerFileWritingUtils.getFilesInDataDirectory();
   }

   public static final String DATA_COLLECTION_FILE_NAME_PREFIX = "autoCollectedScan";
   private long numberOfSavedScans = 0;
   private final int indexMax;
   private double[][] points;
   private final ArrayList<Point3D_F64> scanData;
   private int index = 0;
   private Object notifier = new Object();

   public LidarDataRecorder(int indexMax)
   {
      this.indexMax = indexMax;
      scanData = new ArrayList<Point3D_F64>(indexMax);
      points = new double[indexMax][XYZ_VECTOR_LENGTH];

      for (int i = 0; i < indexMax; i++)
      {
         scanData.add(new Point3D_F64());
      }

      ThreadTools.startAsDaemon(this.new AlgorithmDaemon(), "Lidar Data Recorder Algorithm Daemon");
   }

   protected void addPoint(Point3d point)
   {
      synchronized (notifier)
      {
         point.get(points[index++]);

         if (index >= indexMax)
         {
            index = 0;
            notifier.notify();
         }
      }
   }

   private void copyData()
   {
      for (int i = 0; i < indexMax; i++)
      {
         scanData.get(i).set(points[i][0], points[i][1], points[i][2]);
      }
   }

   private void saveDataToFile()
   {
      String fileName = DATA_COLLECTION_FILE_NAME_PREFIX + (new Date()).toString().replaceAll(":", "") + this.numberOfSavedScans + FILE_EXTENSION;
      try
      {
         CarLocalizerFileWritingUtils.writeToFileInSrcDataDirectory(fileName, scanData);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      catch (URISyntaxException e)
      {
         e.printStackTrace();
      }

      this.numberOfSavedScans++;
   }

   private class AlgorithmDaemon implements Runnable
   {
      public void run()
      {
         while (true)
         {
            synchronized (notifier)
            {
               try
               {
                  notifier.wait();
                  copyData();
               }
               catch (InterruptedException e)
               {
                  e.printStackTrace();
               }
            }

            saveDataToFile();
         }
      }
   }


   public static String getDataFile(int logToDisplay)
   {
      if (logToDisplay < 0)
         logToDisplay = 0;
      if (logToDisplay >= fileNames.length)
         logToDisplay = fileNames.length - 1;

      return fileNames[logToDisplay];
   }
}
