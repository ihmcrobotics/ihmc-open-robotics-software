package us.ihmc.robotEnvironmentAwareness.ui.io;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotics.PlanarRegionFileTools;

public class StereoVisionPointCloudDataExporter
{
   private static final long recodingFrequency = 50;
   private static final int numberOfPointsToSave = 200000;

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExceptionHandling.CATCH_AND_REPORT);

   private final AtomicReference<StereoVisionPointCloudMessage> stereovisionPointCloudMessage;
   private final AtomicReference<String> dataDirectoryPath;

   private final AtomicReference<Boolean> enableRecoding;

   public StereoVisionPointCloudDataExporter(REAUIMessager uiMessager)
   {
      stereovisionPointCloudMessage = uiMessager.createInput(REAModuleAPI.StereoVisionPointCloudState);
      dataDirectoryPath = uiMessager.createInput(REAModuleAPI.UIStereoDataExporterDirectory, new File("Data/").getAbsolutePath());

      enableRecoding = uiMessager.createInput(REAModuleAPI.UIStereoDataExportRequest, false);

      executorService.scheduleAtFixedRate(this::recoding, 0, recodingFrequency, TimeUnit.MILLISECONDS);
   }

   private void recoding()
   {
      if (enableRecoding.get())
      {
         StereoVisionPointCloudMessage message = stereovisionPointCloudMessage.get();
         System.out.println("saving " + " " + message.getPointCloud().size() + " " + message.getRobotTimestamp());

         int numberOfPoints = message.getColors().size();

         Point3D[] pointCloud = new Point3D[numberOfPoints];
         for (int i = 0; i < numberOfPoints; i++)
         {
            pointCloud[i] = new Point3D();
            MessageTools.unpackScanPoint(message, i, pointCloud[i]);
         }

         Path path = Paths.get(dataDirectoryPath.get());
         File file = new File(path.toFile(), "stereovision_pointcloud_" + PlanarRegionFileTools.getDate() + "_" + message.getRobotTimestamp() + ".txt");
         FileWriter fileWriter;
         try
         {
            fileWriter = new FileWriter(file);
            StringBuilder builder = new StringBuilder("");
            for (int i = 0; i < numberOfPoints; i++)
            {
               Point3D scanPoint = pointCloud[i];
               builder.append(i + "\t" + scanPoint.getX() + "\t" + scanPoint.getY() + "\t" + scanPoint.getZ() + "\t" + 0 + "\n");
            }
            fileWriter.write(builder.toString());
            fileWriter.close();
         }
         catch (IOException e1)
         {
            e1.printStackTrace();
         }
      }
   }
}
