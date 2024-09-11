package us.ihmc.rdx.perception;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opencl.OpenCLPointCloudExtractor;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.function.Supplier;

public class SensorTester
{
   private static final int CROSS_SECTION_SIZE = 1000;

   private final String sensorName;
   private final Supplier<RawImage> nextDepthImageSupplier;
   private final int numberOfSamples;

   private final List<Double> centerDepthValues = new ArrayList<>();
   private final List<RawImage> crossSectionImages = new ArrayList<>();

   public SensorTester(String sensorName, Supplier<RawImage> nextDepthImageSupplier, int numberOfSamples)
   {
      this.sensorName = sensorName;
      this.nextDepthImageSupplier = nextDepthImageSupplier;
      this.numberOfSamples = numberOfSamples;
   }

   public void run()
   {
      LogTools.info("Starting {} Test", sensorName);

      Mat crossSectionExtractor = null;

      for (int i = 0; i < numberOfSamples; ++i)
      {
         LogTools.info("\tGetting image {}", i);
         RawImage depthImage = nextDepthImageSupplier.get();

         int centerX = Math.round(depthImage.getPrincipalPointX());
         int centerY = Math.round(depthImage.getPrincipalPointY());

         Mat depthMat = depthImage.getCpuImageMat();

         if (crossSectionExtractor == null)
         {
            crossSectionExtractor = new Mat(depthMat.size(), depthMat.type(), new Scalar(0));
            Point topLeft = new Point(centerX, centerY - CROSS_SECTION_SIZE / 2);
            Point bottomRight = new Point(centerX, centerY + CROSS_SECTION_SIZE / 2);

            opencv_imgproc.rectangle(crossSectionExtractor, topLeft, bottomRight, new Scalar(65535), -1, opencv_imgproc.LINE_4, 0);
         }

         Mat depthCrossSection = new Mat();
         opencv_core.bitwise_and(crossSectionExtractor, depthMat, depthCrossSection);
         crossSectionImages.add(depthImage.replaceImage(depthCrossSection));

         char depthVal = depthMat.col(centerX).row(centerY).data().getChar();
         double depthInMeters = depthVal * depthImage.getDepthDiscretization();
         centerDepthValues.add(depthInMeters);
         LogTools.info("\tDepth: {}m", depthInMeters);

         depthImage.release();
      }

      LogTools.info("Ending {} Test", sensorName);

   }

   public List<Point3D32> getCrossSectionPointCloud()
   {
      OpenCLPointCloudExtractor pointCloudExtractor = new OpenCLPointCloudExtractor();
      List<Point3D32> crossSectionPoints = new ArrayList<>();

      for (RawImage image : crossSectionImages)
      {
         List<Point3D32> pointCloud = pointCloudExtractor.extractPointCloud(image);
         crossSectionPoints.addAll(pointCloud);
      }

      pointCloudExtractor.destroy();

      LogTools.info("Points: {}", crossSectionPoints.size());

      return crossSectionPoints;
   }

   public void saveDataToFile(Path fileDirectory) throws IOException
   {
      String fileName = sensorName + "SensorTest" + new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date()) + ".csv";
      File file = fileDirectory.resolve(fileName).toFile();

      if (file.exists())
         file.delete();

      file.createNewFile();

      BufferedWriter writer = new BufferedWriter(new FileWriter(file));
      writer.write("Depth Distances (m)\n");

      for (double distance : centerDepthValues)
      {
         writer.write(distance + "\n");
      }
      writer.close();
   }

   public void destroy()
   {
      for (RawImage image : crossSectionImages)
      {
         image.release();
      }
   }
}
