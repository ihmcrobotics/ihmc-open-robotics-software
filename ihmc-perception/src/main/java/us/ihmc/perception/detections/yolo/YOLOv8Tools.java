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
import java.util.Locale;

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
      if (pointCloud.isEmpty())
         return pointCloud;

      Point3D32 centroid = new Point3D32();
      double standardDeviation = calculateStandardDeviationAndCentroid(pointCloud, numberOfSamples, true, centroid);
      double inverseStdDev = 1.0 / standardDeviation;

      return pointCloud.parallelStream().filter(point ->
      {
         Vector3D zVector = new Vector3D(point);
         zVector.sub(centroid);
         zVector.scale(inverseStdDev);
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
      if (pointCloud == null || pointCloud.isEmpty())
         return 0.0;

      if (shuffle)
         Collections.shuffle(pointCloud);

      int n = Math.min(pointCloud.size(), maxNumberOfSamples);

      // Welford variables for each axis
      double meanX = 0, meanY = 0, meanZ = 0;
      double m2X = 0, m2Y = 0, m2Z = 0;

      // Welford's Algorithm
      for (int i = 1; i <= n; i++)
      {
         Point3DReadOnly point = pointCloud.get(i - 1);

         double dx = point.getX() - meanX;
         meanX += dx / i;
         m2X += dx * (point.getX() - meanX);

         double dy = point.getY() - meanY;
         meanY += dy / i;
         m2Y += dy * (point.getY() - meanY);

         double dz = point.getZ() - meanZ;
         meanZ += dz / i;
         m2Z += dz * (point.getZ() - meanZ);
      }

      centroidToPack.set(meanX, meanY, meanZ);

      // Calculate Variance (m2 / n)
      // We use population variance (divide by n) to match your previous logic
      double varX = m2X / n;
      double varY = m2Y / n;
      double varZ = m2Z / n;

      // Compute combined Standard Deviation
      // Clamping to 0 just in case of extreme floating point jitter
      double totalVariance = Math.max(0, varX) + Math.max(0, varY) + Math.max(0, varZ);
      return Math.sqrt(totalVariance);
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

   private static Scalar colorForId(int id)
   {
      if (id < 0)
         return GREEN;

      int r = (id * 37) % 255;
      int g = (id * 17) % 255;
      int b = (id * 97) % 255;

      return new Scalar(b, g, r, 255.0);
   }

   /**
    * Annotates the {@code inputImage} using the {@code detections} and puts the result in {@code annotatedImage}.
    *
    * @param inputImage     Image on which YOLO was run on.
    * @param annotatedImage Annotated output Mat.
    * @param detections     YOLO detections.
    */
   public static void annotateImage(Mat inputImage, Mat annotatedImage, List<YOLOv8InstantDetection> detections)
   {
      Mat greenMat = GREEN_MAT.get();
      if (!OpenCVTools.dimensionsMatch(inputImage, greenMat))
         opencv_imgproc.resize(greenMat, greenMat, inputImage.size());

      inputImage.copyTo(annotatedImage);

      for (YOLOv8InstantDetection detection : detections)
      {

         int trackId = detection.getTrackId(); // assume -1 if none
         String text = (trackId >= 0)
               ? String.format("ID:%d %s %.2f", trackId, detection.getDetectedObjectClass(), detection.getConfidence())
               : String.format("%s %.2f", detection.getDetectedObjectClass(), detection.getConfidence());

         // Draw the bounding box
         Rect boundingBox = new Rect((int) Math.round(detection.getBoundingBox().getMinX()),
                                     (int) Math.round(detection.getBoundingBox().getMinY()),
                                     (int) Math.round(detection.getBoundingBox().getMaxX() - detection.getBoundingBox().getMinX()),
                                     (int) Math.round(detection.getBoundingBox().getMaxY() - detection.getBoundingBox().getMinY()));
//         opencv_imgproc.rectangle(annotatedImage, boundingBox, GREEN, 5, LINE_TYPE, 0);

         Scalar color = colorForId(trackId);

         opencv_imgproc.rectangle(annotatedImage, boundingBox, color, 5, LINE_TYPE, 0);

         // Draw text background
         Size textSize = opencv_imgproc.getTextSize(text, FONT, FONT_SCALE, FONT_THICKNESS, new IntPointer());

         int textBoxClampedX = MathTools.clamp(boundingBox.x(), 0, annotatedImage.cols() - textSize.width());
         int textBoxClampedY = MathTools.clamp(boundingBox.y() - textSize.height(), 0, annotatedImage.rows() - textSize.height());

         Rect textBox = new Rect(textBoxClampedX, textBoxClampedY, textSize.width(), textSize.height());
//         opencv_imgproc.rectangle(annotatedImage, textBox, GREEN, opencv_imgproc.FILLED, LINE_TYPE, 0);
         opencv_imgproc.rectangle(annotatedImage, textBox, color, opencv_imgproc.FILLED, LINE_TYPE, 0);

         // Draw the text
         Point textLocation = new Point(textBoxClampedX, textBoxClampedY + textSize.height());
         opencv_imgproc.putText(annotatedImage, text, textLocation, FONT, FONT_SCALE, WHITE, FONT_THICKNESS, LINE_TYPE, false);

         // Add green tint to show mask
         RawImage mask = detection.getObjectMask();
         if (mask != null)
         {
            Mat maskMat = mask.getCpuImageMat();
            if (maskMat != null && !maskMat.isNull())
            {
               // Ensure mask matches frame size
               Mat maskAligned = maskMat;
               Mat resizedMask = null;

               if (maskMat.cols() != annotatedImage.cols() || maskMat.rows() != annotatedImage.rows())
               {
                  resizedMask = new Mat();
                  opencv_imgproc.resize(maskMat,
                                        resizedMask,
                                        annotatedImage.size(),
                                        0, 0,
                                        opencv_imgproc.INTER_NEAREST);
                  maskAligned = resizedMask;
               }

               // Color overlay image
               Mat colorMat = new Mat(annotatedImage.rows(), annotatedImage.cols(), annotatedImage.type(), color);

               // Add colored tint only where mask is non-zero
               opencv_core.add(annotatedImage, colorMat, annotatedImage, maskAligned, -1);

               colorMat.release();
               if (resizedMask != null) resizedMask.release();
            }
         }

         boundingBox.close();
         textBox.close();
      }
   }

   public static void annotateTargets(Mat image, List<AnnotatedTarget2D> targets)
   {
      for (AnnotatedTarget2D t : targets)
      {
         if (t.bbox == null) continue;

         int x1 = Math.round(t.bbox[0]);
         int y1 = Math.round(t.bbox[1]);
         int x2 = Math.round(t.bbox[2]);
         int y2 = Math.round(t.bbox[3]);

         Scalar color = colorForId(t.targetId); // nice stable per targetId

         opencv_imgproc.rectangle(image, new Point(x1, y1), new Point(x2, y2), color, 2, LINE_TYPE, 0);

         String label = "T" + t.targetId + " (trk " + t.trackId + ") " +
                        String.format("%.2f", t.score) + " " + t.name;

         opencv_imgproc.putText(image, label,
                                new Point(x1, Math.max(0, y1 - 5)),
                                FONT, 0.8, WHITE, 2, LINE_TYPE, false);
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