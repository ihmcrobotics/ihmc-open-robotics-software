package us.ihmc.perception.detections.yolo;

import org.apache.commons.lang3.mutable.MutableObject;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Size;
import perception_msgs.msg.dds.YOLOv8ModelInfo;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.tools.io.resources.ResourceTools;

import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URL;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class YOLOv8Tools
{
   private static final int FONT = opencv_imgproc.FONT_HERSHEY_DUPLEX;
   private static final int FONT_THICKNESS = 2;
   private static final double FONT_SCALE = 1.5;
   private static final int LINE_TYPE = opencv_imgproc.LINE_4;
   private static final Scalar GREEN = new Scalar(0.0, 196.0, 0.0, 255.0);
   private static final Scalar WHITE = new Scalar(255.0, 255.0, 255.0, 255.0);
   private static final ThreadLocal<Mat> GREEN_MAT = ThreadLocal.withInitial(() -> new Mat(1, 1, opencv_core.CV_8UC3, GREEN));

   public static final String CLASS_NAME_FILE_NAME = "class_names.yaml";

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
    *
    * @param pointCloud         The list of points used for calculations
    * @param maxNumberOfSamples Maximum number of points to use for the computation. First N points in the list will be used.
    * @param shuffle            Whether to shuffle the point cloud before computations. Can b used to find approximate values with N points.
    * @param centroidToPack     Point object into which the centroid will be packed
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

   public static Point3D32 computeCentroidOfPointCloud(List<Point3D32> pointCloud, int pointsToAverage)
   {
      int numberOfPointsToUse = Math.min(pointsToAverage, pointCloud.size());

      Point3D32 centroid = new Point3D32();
      for (int i = 0; i < numberOfPointsToUse; i++)
         centroid.add(pointCloud.get(i));
      centroid.scale(1.0 / numberOfPointsToUse);

      return centroid;
   }

   /**
    * Annotates the {@code inputImage} using the {@code detections} and puts the result in {@code annotatedImage}.
    *
    * @param inputImage     Image on which YOLO was run on.
    * @param annotatedImage Annotated output Mat.
    * @param detections     YOLO detections.
    */
   public static void annotateImage(Mat inputImage, Mat annotatedImage, List<YOLOv8Detection> detections)
   {
      Mat greenMat = GREEN_MAT.get();
      if (!OpenCVTools.dimensionsMatch(inputImage, greenMat))
         opencv_imgproc.resize(greenMat, greenMat, inputImage.size());

      inputImage.copyTo(annotatedImage);

      for (YOLOv8Detection detection : detections)
      {
         String text = String.format("%s: %.2f", detection.objectClass(), detection.confidence());

         // Draw the bounding box
         Rect boundingBox = detection.boundingBoxRect();
         opencv_imgproc.rectangle(annotatedImage, boundingBox, GREEN, 5, LINE_TYPE, 0);

         // Draw text background
         Size textSize = opencv_imgproc.getTextSize(text, FONT, FONT_SCALE, FONT_THICKNESS, new IntPointer());

         int textBoxClampedX = MathTools.clamp(boundingBox.x(), 0, annotatedImage.cols() - textSize.width());
         int textBoxClampedY = MathTools.clamp(boundingBox.y() - textSize.height(), 0, annotatedImage.rows() - textSize.height());

         Rect textBox = new Rect(textBoxClampedX, textBoxClampedY, textSize.width(), textSize.height());
         opencv_imgproc.rectangle(annotatedImage, textBox, GREEN, opencv_imgproc.FILLED, LINE_TYPE, 0);

         // Draw the text
         Point textLocation = new Point(textBoxClampedX, textBoxClampedY + textSize.height());
         opencv_imgproc.putText(annotatedImage, text, textLocation, FONT, FONT_SCALE, WHITE, FONT_THICKNESS, LINE_TYPE, false);

         // Add green tint to show mask
         RawImage mask = detection.mask();
         Mat maskMat = mask.getCpuImageMat();

         // Account for aspect ratio by scaling to match annotatedImage width
         Size scaleSize = annotatedImage.rows() > maskMat.rows() ?
                                new Size(annotatedImage.cols(), maskMat.rows() * annotatedImage.cols() / maskMat.cols())
                              : new Size(maskMat.cols() * annotatedImage.rows() / maskMat.rows(), annotatedImage.rows());
         Mat scaledMask = new Mat(scaleSize, maskMat.type());
         opencv_imgproc.resize(maskMat, scaledMask, scaleSize);
         Scalar scalar = new Scalar(0);
         Mat paddedMask = new Mat(annotatedImage.rows(), annotatedImage.cols(), maskMat.type(), scalar);
         Rect roi = annotatedImage.rows() > maskMat.rows() ?
                          new Rect(0, (annotatedImage.rows() - scaledMask.rows()) / 2, scaledMask.cols(), scaledMask.rows())
                        : new Rect((annotatedImage.cols() - scaledMask.cols()) / 2, 0, scaledMask.cols(), scaledMask.rows());
         Mat paddedMaskCenter = new Mat(paddedMask, roi);
         scaledMask.copyTo(paddedMaskCenter);
         scaleSize.close();
         paddedMaskCenter.close();
         scalar.close();
         roi.close();

         opencv_core.add(annotatedImage, greenMat, annotatedImage, paddedMask, -1);
         paddedMask.close();

         boundingBox.close();
         textBox.close();
         mask.release();
      }
   }

   public static List<URL> getYOLOModelDirectories(URL baseModelsDirectory)
   {
      if (baseModelsDirectory == null)
         throw new NullPointerException("Base YOLO Model Directory is NULL");

      List<URL> directories = new ArrayList<>();
      try
      {
         ResourceTools.processAsPath(baseModelsDirectory, modelsDirectoryPath ->
         {
            try (DirectoryStream<Path> stream = Files.newDirectoryStream(modelsDirectoryPath))
            {
               for (Path directory : stream)
               {
                  if (isValidYOLOModelDirectory(directory))
                     directories.add(new URL(baseModelsDirectory, directory.getFileName().toString() + "/"));
               }
            }
            catch (MalformedURLException exception)
            {
               throw new RuntimeException(exception);
            }
         });
      }
      catch (IOException exception)
      {
         throw new RuntimeException(exception);
      }

      return directories;
   }

   public static List<URL> getYOLOModelDirectories()
   {
      return getYOLOModelDirectories(YOLOv8Tools.class.getResource("/yolo/"));
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

   public static URL getONNXFile(URL yoloModelDirectory)
   {
      if (yoloModelDirectory == null)
         throw new NullPointerException("YOLO Model Directory is NULL");

      MutableObject<URL> onnxFileURL = new MutableObject<>(null);
      try
      {
         ResourceTools.processAsPath(yoloModelDirectory, directoryPath ->
         {
            try (Stream<Path> directoryContents = Files.list(directoryPath))
            {
               Optional<Path> onnxFile = directoryContents.filter(path -> path.getFileName().toString().endsWith(".onnx")).findAny();
               if (onnxFile.isPresent())
                  onnxFileURL.setValue(onnxFile.get().toUri().toURL());
            }
            catch (MalformedURLException exception)
            {
               throw new RuntimeException(exception);
            }
         });
      }
      catch (IOException ioException)
      {
         throw new RuntimeException(ioException);
      }

      if (onnxFileURL.getValue() == null)
         throw new IllegalArgumentException("Could not find an onnx file in %s".formatted(yoloModelDirectory.toString()));

      return onnxFileURL.getValue();
   }

   public static URL getClassNamesFile(URL yoloModelDirectory)
   {
      if (yoloModelDirectory == null)
         throw new NullPointerException("YOLO Model Directory is NULL");

      MutableObject<URL> classNamesURL = new MutableObject<>(null);
      try
      {
         ResourceTools.processAsPath(yoloModelDirectory, directoryPath ->
         {
            try (Stream<Path> directoryContents = Files.list(directoryPath))
            {
               Optional<Path> classNamesFile = directoryContents.filter(path -> path.getFileName().toString().endsWith(CLASS_NAME_FILE_NAME)).findAny();
               if (classNamesFile.isPresent())
                  classNamesURL.setValue(classNamesFile.get().toUri().toURL());
            }
            catch (IOException ioException)
            {
               throw new RuntimeException(ioException);
            }
         });
      }
      catch (IOException ioException)
      {
         throw new RuntimeException(ioException);
      }

      if (classNamesURL.getValue() == null)
         throw new IllegalArgumentException("Could not find an class names file in %s".formatted(yoloModelDirectory.toString()));

      return classNamesURL.getValue();
   }

   public static void toMessage(YOLOv8Model model, YOLOv8ModelInfo messageToPack)
   {
      messageToPack.setModelName(model.getName());
      messageToPack.getDetectableObjectClasses().clear();
      model.getDetectableObjects().forEach(detectableObject -> messageToPack.getDetectableObjectClasses().add(detectableObject));
   }

   public static YOLOv8ModelInfo toMessage(YOLOv8Model model)
   {
      YOLOv8ModelInfo modelInfo = new YOLOv8ModelInfo();
      toMessage(model, modelInfo);
      return modelInfo;
   }
}