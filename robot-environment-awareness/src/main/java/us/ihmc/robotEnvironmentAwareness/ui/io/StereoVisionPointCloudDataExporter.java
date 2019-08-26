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
   private static final long recodingFrequency = 500;
   private static final int numberOfPointsToSave = 200000;

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExceptionHandling.CATCH_AND_REPORT);

   private final AtomicReference<StereoVisionPointCloudMessage> stereovisionPointCloudMessage;
   private final AtomicReference<String> dataDirectoryPath;

   private final AtomicReference<Boolean> enableRecording;

   public StereoVisionPointCloudDataExporter(REAUIMessager uiMessager)
   {
      stereovisionPointCloudMessage = uiMessager.createInput(REAModuleAPI.StereoVisionPointCloudState);
      dataDirectoryPath = uiMessager.createInput(REAModuleAPI.UIStereoDataExporterDirectory, new File("Data/").getAbsolutePath());

      enableRecording = uiMessager.createInput(REAModuleAPI.UIStereoDataExportRequest, false);

      executorService.scheduleAtFixedRate(this::recording, 0, recodingFrequency, TimeUnit.MILLISECONDS);
   }

   private void recording()
   {
      if (enableRecording.get())
      {
         StereoVisionPointCloudMessage message = stereovisionPointCloudMessage.get();
         int numberOfPoints = message.getColors().size();

         Point3D[] pointCloud = new Point3D[numberOfPoints];
         for (int i = 0; i < numberOfPoints; i++)
         {
            pointCloud[i] = new Point3D();
            MessageTools.unpackScanPoint(message, i, pointCloud[i]);
         }

         Path path = Paths.get(dataDirectoryPath.get());
         File file = new File(path.toFile(), "stereovision_pointcloud_" + PlanarRegionFileTools.getDate() + "_" + message.robot_timestamp_ + ".txt");
         FileWriter fileWriter;
         try
         {
            fileWriter = new FileWriter(file);
            StringBuilder builder = new StringBuilder("");
            for (int i = 0; i < numberOfPoints; i++)
            {
               Point3D scanPoint = pointCloud[i];
               int color =  message.getColors().get(i);
               builder.append(i + "\t" + scanPoint.getX() + "\t" + scanPoint.getY() + "\t" + scanPoint.getZ() + "\t" + color + "\n");
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
