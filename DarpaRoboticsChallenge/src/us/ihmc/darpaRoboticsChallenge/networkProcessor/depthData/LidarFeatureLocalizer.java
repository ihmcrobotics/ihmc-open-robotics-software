package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;

import java.io.IOException;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;

import us.ihmc.darpaRoboticsChallenge.odometry.CarLocalizerFileWritingUtils;
import us.ihmc.darpaRoboticsChallenge.odometry.IcpCloud3D;
import us.ihmc.tools.thread.ThreadTools;

/**
 * @author GrayThomas
 *
 */
public class LidarFeatureLocalizer
{
   private static final boolean TRANSFORM_LOGS = false;
   private static final String FILE_EXTENSION = ".dat";
   private static final String LOG_NAME_PREFIX = "loggedScan";
   private static final int XYZ_VECTOR_LENGTH = 3;
   private final boolean loggingEnabled;
   private long numberOfSavedScans = 0;
   private final int indexMax;
   private double[][] points;
   private final ArrayList<Point3D_F64> scanData;
   private final ArrayList<Point3D_F64> logData;
   private final List<Point3D_F64> template;
   private final GeoregressionTransformListener georegressionTransformListener;
   private final IcpCloud3D iterativeCPSolver;
   private final String name;
   private int index = 0;
   private Object notifier = new Object();

   public LidarFeatureLocalizer(String name, GeoregressionTransformListener georegressionTransformListener, String templateResourceName, boolean enableLogging, int indexMax,
                                IcpCloud3D iterativeCPSolver)
   {
      this.indexMax = indexMax;
      scanData = new ArrayList<Point3D_F64>(indexMax);
      logData = new ArrayList<Point3D_F64>(indexMax);
      points = new double[indexMax][XYZ_VECTOR_LENGTH];

      for (int i = 0; i < indexMax; i++)
      {
         scanData.add(new Point3D_F64());
      }

      for (int i = 0; i < indexMax; i++)
      {
         logData.add(new Point3D_F64());
      }

      this.template = loadTemplate(templateResourceName);
      System.out.println("DEBUG LidarFeatureLocalizer: this.template has " + template.size() + " entries.");

//    this.iterativeCPSolver = new IcpCloud3D(0.2, 30, 1e-8);
      this.iterativeCPSolver = iterativeCPSolver;
      this.iterativeCPSolver.setReference(this.template);

      this.loggingEnabled = enableLogging;
      this.georegressionTransformListener = georegressionTransformListener;
      this.name = name;

      ThreadTools.startAsDaemon(this.new AlgorithmDaemon(), "Lidar Feature Localizer Algorithm Daemon");
   }

   protected List<Point3D_F64> loadTemplate(String templateResourceName)
   {
      List<Point3D_F64> template = new ArrayList<Point3D_F64>();
      try
      {
         template = CarLocalizerFileWritingUtils.readJARResource("data/" + templateResourceName);
      }
      catch (Exception e)
      {
         System.err.println("Failed to acquire template for Lidar Car Localizer. " + templateResourceName);
         e.printStackTrace();
      }

      return template;
   }

   protected void addPoint(Point3d point)
   {
      synchronized (notifier)
      {
         if (scans<=0)
            return;
         
         point.get(points[index++]);

         if (index >= indexMax)
         {
            index = 0;
            scans--;
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

      if (loggingEnabled)
      {
         for (int i = 0; i < indexMax; i++)
         {
            logData.get(i).set(points[i][0], points[i][1], points[i][2]);
         }
      }
   }

   private void processDataToProduceEstimate()
   {
      List<Point3D_F64> scanData = this.scanData;
      Se3_F64 foundTransformFromCurrentToTemplate = calculateTransformFromCurrentToTemplate(scanData);
      if (foundTransformFromCurrentToTemplate == null)
         return;
      if (loggingEnabled)
         saveLogFile(foundTransformFromCurrentToTemplate);
      georegressionTransformListener.handleTransform(foundTransformFromCurrentToTemplate);

   }

   protected Se3_F64 calculateTransformFromCurrentToTemplate(List<Point3D_F64> scanData)
   {
      if (!iterativeCPSolver.setCurrent(scanData))
      {
         System.err.println("LidarCarLocalizer: Iterative Closest Point Solver failed.");

         return null;
      }

      Se3_F64 transformFromCurrentToTemplate = iterativeCPSolver.getReferenceToCurrent();

      return transformFromCurrentToTemplate;
   }

   private void saveLogFile(Se3_F64 transformFromPelvisToBasePelvis)
   {
      long number = this.numberOfSavedScans;
      String fileName = getLogName(number, this.name);
      for (int i = 0; i < logData.size(); i++)
      {
         if (TRANSFORM_LOGS)

            SePointOps_F64.transformReverse(transformFromPelvisToBasePelvis, logData.get(i), scanData.get(i));
         else
            scanData.get(i).set(logData.get(i));

      }

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

   public static String getLogName(long number, String name)
   {
      return LOG_NAME_PREFIX + name + number + FILE_EXTENSION;
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
            System.out.println("LidarFeatureLocalizer: computing transform");
            processDataToProduceEstimate();
         }
      }
   }
   private int scans = 0;
   public void scanOneMoreTime()
   {
      this.scans++;
      
   }
}
