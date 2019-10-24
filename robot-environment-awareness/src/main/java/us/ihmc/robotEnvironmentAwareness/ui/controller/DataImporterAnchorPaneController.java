package us.ihmc.robotEnvironmentAwareness.ui.controller;

import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.Image32;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.stage.Window;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.LidarImageFusionDataLoader;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;

public class DataImporterAnchorPaneController
{
   private Window window;
   private REAUIMessager reaMessager;
   private JavaFXMessager messager = null;

   private static final String imageFileNameHeader = "image_";
   private static final String stereoDataFileNameHeader = "stereovision_pointcloud_";
   private static final String imageFileExtension = ".jpg";
   private static final String stereoDataFileExtension = ".txt";

   private static final int DEFAULT_STEREO_POINT_COLOR = 0x00FF00;

   private final List<BufferedImage> loadedImages = new ArrayList<BufferedImage>();
   private final List<Point3D[]> loadedStereoData = new ArrayList<Point3D[]>();

   @FXML private TextField tfDataIndex;
   @FXML private TextField tfRollDegree;
   @FXML private TextField tfPitchDegree;
   @FXML private TextField tfYawDegree;

   public void initialize(REAUIMessager reaMessager, SharedMemoryJavaFXMessager messager, Window window)
   {
      this.reaMessager = reaMessager;
      if (messager != null)
         this.messager = messager;
      this.window = window;
   }

   public void loadFromFile()
   {
      String dataFileRootPath = PlanarRegionDataImporter.chooseFile(window).getAbsolutePath() + "\\";
      System.out.println("file root path " + dataFileRootPath);

      loadedImages.clear();
      loadedStereoData.clear();
      int numberOfFiles = 0;
      while (true)
      {
         String imageFilePath = dataFileRootPath + imageFileNameHeader + numberOfFiles + imageFileExtension;
         String stereoDataFilePath = dataFileRootPath + stereoDataFileNameHeader + numberOfFiles + stereoDataFileExtension;

         File imageFile = new File(imageFilePath);
         File stereoDataFile = new File(stereoDataFilePath);

         boolean fileIsAvailable = imageFile.canRead() && stereoDataFile.canRead();
         if (fileIsAvailable)
         {
            loadedImages.add(LidarImageFusionDataLoader.readImage(imageFile));
            loadedStereoData.add(LidarImageFusionDataLoader.readPointCloud(stereoDataFile));
            numberOfFiles++;
         }
         else
         {
            System.out.println("The Number Of Files is " + numberOfFiles);
            break;
         }
      }
   }

   public void publishMessages()
   {
      int dataIndex = Integer.parseInt(tfDataIndex.getText());

      double roll = Double.parseDouble(tfRollDegree.getText()) / 180 * Math.PI;
      double pitch = Double.parseDouble(tfPitchDegree.getText()) / 180 * Math.PI;
      double yaw = Double.parseDouble(tfYawDegree.getText()) / 180 * Math.PI;

      if (dataIndex < 0 || dataIndex >= loadedStereoData.size())
      {
         System.out.println("Put proper index to publish.");
         return;
      }

      RigidBodyTransform manualTransform = new RigidBodyTransform();

      /**
       * tripod is yaw pitch roll.
       */
      manualTransform.appendYawRotation(yaw);
      manualTransform.appendPitchRotation(pitch);
      manualTransform.appendRollRotation(roll);

      Point3D[] pointCloud = loadedStereoData.get(dataIndex);
      int numberOfPoints = pointCloud.length;

      long timestamp = 870612L;
      float[] pointCloudBuffer = new float[3 * numberOfPoints];
      int[] colorsInteger = new int[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D scanPoint = new Point3D(pointCloud[i]);
         manualTransform.transform(scanPoint);

         pointCloudBuffer[3 * i + 0] = (float) scanPoint.getX();
         pointCloudBuffer[3 * i + 1] = (float) scanPoint.getY();
         pointCloudBuffer[3 * i + 2] = (float) scanPoint.getZ();

         colorsInteger[i] = DEFAULT_STEREO_POINT_COLOR;
      }

      StereoVisionPointCloudMessage dummyMessage = MessageTools.createStereoVisionPointCloudMessage(timestamp, pointCloudBuffer, colorsInteger);
      reaMessager.submitMessageToModule(REAModuleAPI.StereoVisionPointCloudState, dummyMessage);

      if (messager != null)
      {
         messager.submitMessage(LidarImageFusionAPI.CameraOrientationState, new Quaternion(manualTransform.getRotation()));

         BufferedImage bufferedImage = loadedImages.get(dataIndex);
         int height = bufferedImage.getHeight();
         int width = bufferedImage.getWidth();
         Image32 message = new Image32();
         message.setHeight(height);
         message.setWidth(width);
         for (int i = 0; i < height; i++)
         {
            for (int j = 0; j < width; j++)
            {
               message.getRgbdata().add(bufferedImage.getRGB(j, i));
            }
         }
         messager.submitMessage(LidarImageFusionAPI.ImageState, message);
      }
   }

   public void exportPlanarRegions()
   {
      reaMessager.submitMessageInternal(REAModuleAPI.UIPlanarRegionDataExportRequest, true);
   }
}
