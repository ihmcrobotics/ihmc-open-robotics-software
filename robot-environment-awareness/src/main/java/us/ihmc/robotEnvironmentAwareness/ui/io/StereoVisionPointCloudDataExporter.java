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
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.idl.IDLSequence.Integer;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;

public class StereoVisionPointCloudDataExporter
{
   private static final long recodingFrequency = 500;

   public static final String POINT_CLOUD_FILE_NAME_HEADER = "stereo";
   public static final String SENSOR_POSE_FILE_NAME_HEADER = "pose";
   public static final String STEREO_DATA_SPLITER = "_";
   public static final String STEREO_DATA_EXTENSION = ".txt";

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

         saveSensorPose(path, message.timestamp_, message.getSensorPosition(), message.getSensorOrientation());
         savePointCloud(path, message.timestamp_, pointCloud, message.getColors());
      }
   }

   private static void saveSensorPose(Path path, long timestamp, Point3D sensorPosition, Quaternion sensorOrientation)
   {
      File sensorPoseFile = new File(path.toFile(), SENSOR_POSE_FILE_NAME_HEADER + STEREO_DATA_SPLITER + timestamp + STEREO_DATA_EXTENSION);
      FileWriter sensorPoseFileWriter;
      try
      {
         sensorPoseFileWriter = new FileWriter(sensorPoseFile);
         StringBuilder builder = new StringBuilder("");
         builder.append(sensorPosition.getX() + "\t" + sensorPosition.getY() + "\t" + sensorPosition.getZ() + "\t");
         builder.append(sensorOrientation.getX() + "\t" + sensorOrientation.getY() + "\t" + sensorOrientation.getZ() + "\t" + sensorOrientation.getS());
         sensorPoseFileWriter.write(builder.toString());
         sensorPoseFileWriter.close();
      }
      catch (IOException e1)
      {
         e1.printStackTrace();
      }
   }

   private static void savePointCloud(Path path, long timestamp, Point3D[] pointCloud, Integer colors)
   {
      File pointCloudFile = new File(path.toFile(), POINT_CLOUD_FILE_NAME_HEADER + STEREO_DATA_SPLITER + timestamp + STEREO_DATA_EXTENSION);
      FileWriter pointCloudFileWriter;
      try
      {
         pointCloudFileWriter = new FileWriter(pointCloudFile);
         StringBuilder builder = new StringBuilder("");
         for (int i = 0; i < colors.size(); i++)
         {
            Point3D scanPoint = pointCloud[i];
            int color = colors.get(i);
            builder.append(i + "\t" + scanPoint.getX() + "\t" + scanPoint.getY() + "\t" + scanPoint.getZ() + "\t" + color + "\n");
         }
         pointCloudFileWriter.write(builder.toString());
         pointCloudFileWriter.close();
      }
      catch (IOException e1)
      {
         e1.printStackTrace();
      }
   }
}
