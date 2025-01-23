package us.ihmc.perception.detections.yolo;

import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.RawImage;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class YOLOv8Tools
{
   public static final String CLASS_NAME_FILE_NAME = "class_names.yaml";
//   private static final WorkspaceResourceDirectory POINT_CLOUD_DIRECTORY = new WorkspaceResourceDirectory(YOLOv8DetectionClass.class, "/yoloICPPointClouds/");

   public static List<Point3D32> filterOutliers(List<Point3D32> pointCloud, double zScoreThreshold, int numberOfSamples)
   {
      Point3D32 centroid = new Point3D32();
      double standardDeviation = calculateStandardDeviationAndCentroid(pointCloud, numberOfSamples, true, centroid);

      return pointCloud.parallelStream().filter(point ->
      {
         Vector3D zVector = new Vector3D(point);
         zVector.sub(centroid);
         zVector.scale(1.0 / standardDeviation);
         return zVector.norm() < zScoreThreshold;
      }).collect(Collectors.toList());
   }

   public static double calculateStandardDeviationAndCentroid(List<? extends Point3DReadOnly> pointCloud, Point3DBasics centroidToPack)
   {
      return calculateStandardDeviationAndCentroid(pointCloud, pointCloud.size(), false, centroidToPack);
   }

   /**
    * Given a point cloud, computes the centroid and standard deviation of the points
    * @param pointCloud          The list of points used for calculations
    * @param maxNumberOfSamples  Maximum number of points to use for the computation. First N points in the list will be used.
    * @param shuffle             Whether to shuffle the point cloud before computations. Can b used to find approximate values with N points.
    * @param centroidToPack      Point object into which the centroid will be packed
    * @return The standard deviation of the points
    */
   public static double calculateStandardDeviationAndCentroid(List<? extends Point3DReadOnly> pointCloud,
                                                              int maxNumberOfSamples,
                                                              boolean shuffle,
                                                              Point3DBasics centroidToPack)
   {
      if (shuffle)
         Collections.shuffle(pointCloud);

      Vector3D sumVector = new Vector3D(0.0, 0.0, 0.0);
      Vector3D squaredSumVector = new Vector3D(0.0, 0.0, 0.0);
      int numberOfSamples = Math.min(pointCloud.size(), maxNumberOfSamples);

      pointCloud.parallelStream().limit(numberOfSamples).forEach(point ->
                                                                 {
                                                                    sumVector.add(point);
                                                                    squaredSumVector.add((point.getX() * point.getX()), (point.getY() * point.getY()), (point.getZ() * point.getZ()));
                                                                 });

      centroidToPack.set(sumVector);
      centroidToPack.scale(1.0 / numberOfSamples);

      Vector3D meanSquaredVector = new Vector3D(centroidToPack);
      meanSquaredVector.scale(meanSquaredVector.getX(), meanSquaredVector.getY(), meanSquaredVector.getZ());

      Vector3D varianceVector = new Vector3D(squaredSumVector);
      varianceVector.scale(1.0 / numberOfSamples);
      varianceVector.sub(meanSquaredVector);

      return Math.sqrt(varianceVector.getX() + varianceVector.getY() + varianceVector.getZ());
   }

//   public static List<Point3D32> loadPointCloudFromFile(String fileName)
//   {
//      if (fileName == null)
//         throw new NullPointerException("We can't run ICP on this object yet because we don't have a model point cloud file.");
//
//      WorkspaceResourceFile pointCloudFile = new WorkspaceResourceFile(POINT_CLOUD_DIRECTORY, fileName);
//      List<Point3D32> pointCloud;
//      try (BufferedReader bufferedReader = new BufferedReader(new FileReader(pointCloudFile.getFilesystemFile().toFile())))
//      {
//         pointCloud = bufferedReader.lines().map(line ->
//                                                 {
//                                                    String[] xyzValues = line.split(",");
//                                                    float x = Float.parseFloat(xyzValues[0]);
//                                                    float y = Float.parseFloat(xyzValues[1]);
//                                                    float z = Float.parseFloat(xyzValues[2]);
//                                                    return new Point3D32(x, y, z);
//                                                 }).collect(Collectors.toList());
//      }
//      catch (Exception e)
//      {
//         e.printStackTrace();
//         throw new RuntimeException("Failed trying to load the file.");
//         // Handle any I/O problems
//      }
//
//      return pointCloud;
//   }

   public static Point3D32 computeCentroidOfPointCloud(List<Point3D32> pointCloud, int pointsToAverage)
   {
      int numberOfPointsToUse = Math.min(pointsToAverage, pointCloud.size());

      Point3D32 centroid = new Point3D32();
      for (int i = 0; i < numberOfPointsToUse; i++)
         centroid.add(pointCloud.get(i));
      centroid.scale(1.0 / numberOfPointsToUse);

      return centroid;
   }

   public static void annotateImage(Mat inputImage, Mat annotatedImage, List<YOLOv8Detection> detections)
   {
      int font = opencv_imgproc.FONT_HERSHEY_DUPLEX;
      int lineType = opencv_imgproc.LINE_4;
      double fontScale = 1.5;
      int fontThickness = 2;
      Scalar green = new Scalar(0.0, 196.0, 0.0, 255.0);
      Scalar white = new Scalar(255.0, 255.0, 255.0, 255.0);
      Mat greenMat = new Mat(inputImage.size(), opencv_core.CV_8UC3, green);

      inputImage.copyTo(annotatedImage);

      for (YOLOv8Detection detection : detections)
      {
         String text = String.format("%s: %.2f", detection.name(), detection.confidence());

         // Draw the bounding box
         Rect boundingBox = detection.boundingBox();
         opencv_imgproc.rectangle(annotatedImage, boundingBox, green, 5, lineType, 0);

         // Draw text background
         Size textSize = opencv_imgproc.getTextSize(text, font, fontScale, fontThickness, new IntPointer());

         int textBoxClampedX = MathTools.clamp(boundingBox.x(), 0, annotatedImage.cols() - textSize.width());
         int textBoxClampedY = MathTools.clamp(boundingBox.y() - textSize.height(), 0, annotatedImage.rows() - textSize.height());

         Rect textBox = new Rect(textBoxClampedX, textBoxClampedY, textSize.width(), textSize.height());
         opencv_imgproc.rectangle(annotatedImage, textBox, green, opencv_imgproc.FILLED, lineType, 0);

         // Draw the text
         Point textLocation = new Point(textBoxClampedX, textBoxClampedY + textSize.height());
         opencv_imgproc.putText(annotatedImage, text, textLocation, font, fontScale, white, fontThickness, lineType, false);

         // Add green tint to show mask
         RawImage mask = detection.mask();
         Mat resizedMask = new Mat();
         opencv_imgproc.resize(mask.getCpuImageMat(), resizedMask, annotatedImage.size());
         opencv_core.add(annotatedImage, greenMat, annotatedImage, resizedMask, -1);

         boundingBox.close();
         textBox.close();
         mask.release();
      }

      green.close();
      white.close();
      greenMat.close();
   }

   public static List<Path> getYOLOModelDirectories()
   {
      // Automatically create the yolo-models directory if it doesn't exist
      File yoloModelsDirectoryFile = IHMCCommonPaths.YOLO_MODELS_DIRECTORY.toFile();

      if (!yoloModelsDirectoryFile.exists() || !yoloModelsDirectoryFile.isDirectory())
      {
         yoloModelsDirectoryFile.mkdirs();
      }

      return getYOLOModelDirectories(IHMCCommonPaths.YOLO_MODELS_DIRECTORY);
   }

   public static List<Path> getYOLOModelDirectories(Path baseDirectoryPath)
   {
      try (Stream<Path> directoryContents = Files.list(baseDirectoryPath))
      {
         return directoryContents.filter(YOLOv8Tools::isValidYOLOModelDirectory).toList();
      }
      catch (IOException e)
      {
         return null;
      }
   }

   public static boolean isValidYOLOModelDirectory(Path yoloModelDirectory)
   {
      try (Stream<Path> onnxFiles = Files.list(yoloModelDirectory).filter(path -> path.getFileName().toString().endsWith(".onnx"));
           Stream<Path> classNameFiles = Files.list(yoloModelDirectory).filter(path -> path.getFileName().toString().equals(CLASS_NAME_FILE_NAME)))
      {
         return onnxFiles.count() == 1 && classNameFiles.count() == 1;
      }
      catch (IOException e)
      {
         return false;
      }
   }

   public static Path getONNXFile(Path yoloModelDirectory)
   {
      try (Stream<Path> directoryContents = Files.list(yoloModelDirectory))
      {
         Optional<Path> onnxFile = directoryContents.filter(path -> path.getFileName().toString().endsWith(".onnx")).findAny();
         if (onnxFile.isPresent())
            return onnxFile.get();

         throw new IllegalArgumentException("Could not find an onnx file in %s".formatted(yoloModelDirectory.toString()));
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static Path getClassNamesFile(Path yoloModelDirectory)
   {
      try (Stream<Path> directoryContents = Files.list(yoloModelDirectory))
      {
         Optional<Path> classNamesFile = directoryContents.filter(path -> path.getFileName().endsWith(CLASS_NAME_FILE_NAME)).findAny();
         if (classNamesFile.isPresent())
            return classNamesFile.get();

         throw new IllegalArgumentException("Could not find an class names file in %s".formatted(yoloModelDirectory.toString()));
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
}