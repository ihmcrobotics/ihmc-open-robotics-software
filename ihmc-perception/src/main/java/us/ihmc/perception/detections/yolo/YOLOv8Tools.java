package us.ihmc.perception.detections.yolo;

import org.apache.commons.lang3.mutable.MutableObject;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudawarping;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Size;
import perception_msgs.msg.dds.YOLOv8ModelInfo;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
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
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static org.bytedeco.opencv.opencv_core.AbstractScalar.*;

public class YOLOv8Tools
{
   private static final int FONT = opencv_imgproc.FONT_HERSHEY_DUPLEX;
   private static final int FONT_THICKNESS = 2;
   private static final double FONT_SCALE_SMALL = 1.0;
   private static final double FONT_SCALE = 1.5;
   private static final int TEXT_LINE_TYPE = opencv_imgproc.LINE_AA;
   private static final int LINE_TYPE = opencv_imgproc.LINE_4;
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
      if (numberOfPointsToUse <= 0)
         return new Point3D32();

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

   public static void resizeWithCrop(Mat inputImage, Mat outputImage, Size desiredSize)
   {
      Mat resizedMat = new Mat();

      int desiredWidth = desiredSize.width();
      int desiredHeight = desiredSize.height();
      double scaleFactor = Math.max((double) desiredWidth / inputImage.cols(), (double) desiredHeight / inputImage.rows());

      // Use explicit target dimensions so the resized image always fully covers desiredSize
      int resizedWidth = (int) Math.ceil(inputImage.cols() * scaleFactor);
      int resizedHeight = (int) Math.ceil(inputImage.rows() * scaleFactor);
      opencv_imgproc.resize(inputImage, resizedMat, new Size(resizedWidth, resizedHeight), 0.0, 0.0, opencv_imgproc.INTER_LINEAR);

      int horizontalCrop = Math.max(resizedWidth - desiredWidth, 0) / 2;
      int verticalCrop = Math.max(resizedHeight - desiredHeight, 0) / 2;

      Rect roi = new Rect(horizontalCrop, verticalCrop, desiredWidth, desiredHeight);
      resizedMat.apply(roi).copyTo(outputImage);
      resizedMat.close();
   }

   public static void resizeWithCrop(GpuMat inputImage, GpuMat outputImage, Size desiredSize)
   {
      GpuMat resizedMat = new GpuMat();

      int desiredWidth = desiredSize.width();
      int desiredHeight = desiredSize.height();
      double scaleFactor = Math.max((double) desiredWidth / inputImage.cols(), (double) desiredHeight / inputImage.rows());

      // Use explicit target dimensions so the resized image always fully covers desiredSize
      int resizedWidth = (int) Math.ceil(inputImage.cols() * scaleFactor);
      int resizedHeight = (int) Math.ceil(inputImage.rows() * scaleFactor);
      opencv_cudawarping.resize(inputImage, resizedMat, new Size(resizedWidth, resizedHeight), 0.0, 0.0, opencv_imgproc.INTER_LINEAR, null);

      int horizontalCrop = Math.max(resizedWidth - desiredWidth, 0) / 2;
      int verticalCrop = Math.max(resizedHeight - desiredHeight, 0) / 2;

      Rect roi = new Rect(horizontalCrop, verticalCrop, desiredWidth, desiredHeight);
      resizedMat.apply(roi).copyTo(outputImage);
      resizedMat.close();
   }

   public static float[][] getMaskAsPolygons(Mat mask, float precision)
   {
      float xNormalizer = 1.0f / mask.cols();
      float yNormalizer = 1.0f / mask.rows();

      MatVector contours = new MatVector();
      Mat hierarchy = new Mat();
      opencv_imgproc.findContours(mask, contours, hierarchy, opencv_imgproc.RETR_TREE, opencv_imgproc.CHAIN_APPROX_SIMPLE);

      // Get lower resolution maskPolygons for each contour
      float[][] polygons = new float[(int) contours.size()][];
      for (int i = 0; i < contours.size(); ++i)
      {
         Mat contour = contours.get(i);

         double contourPerimeter = opencv_imgproc.arcLength(contour, true);

         Mat polygonApproximation = new Mat();
         opencv_imgproc.approxPolyDP(contour, polygonApproximation, precision * contourPerimeter, true);

         MatVector polygonChannels = new MatVector();
         opencv_core.split(polygonApproximation, polygonChannels);

         Mat xValues = polygonChannels.get(0);
         Mat yValues = polygonChannels.get(1);

         xValues.convertTo(xValues, opencv_core.CV_32F, xNormalizer, 0.0);
         yValues.convertTo(yValues, opencv_core.CV_32F, yNormalizer, 0.0);

         Mat normalizedPolygonApproximation = new Mat();
         opencv_core.merge(polygonChannels, normalizedPolygonApproximation);

         polygons[i] = new float[2 * normalizedPolygonApproximation.rows()];
         new FloatPointer(normalizedPolygonApproximation.data()).get(polygons[i]);

         normalizedPolygonApproximation.close();
         xValues.close();
         yValues.close();
         polygonChannels.close();
         polygonApproximation.close();
         contour.close();
      }

      hierarchy.close();
      contours.close();

      return polygons;
   }

   /**
    * Annotates the {@code inputImage} using the {@code detections} and puts the result in {@code annotatedImage}.
    *
    * @param inputImage     Image on which YOLO was run on.
    * @param annotatedImage Annotated output Mat.
    * @param detections     YOLO detections.
    */
   /**
    * Annotates the {@code inputImage} using the {@code detections} and puts the result in {@code annotatedImage}.
    *
    * @param inputImage     Image on which YOLO was run on.
    * @param annotatedImage Annotated output Mat.
    * @param detections     YOLO detections.
    */
   public static void annotateImage(Mat inputImage, Mat annotatedImage, List<YOLOv8InstantDetection> detections)
   {
      inputImage.copyTo(annotatedImage);

      for (YOLOv8InstantDetection detection : detections)
      {
         int trackId = detection.getTrackId();
         Scalar color = colorForId(trackId);

         String text = trackId >= 0
               ? String.format(Locale.US, "ID:%d %s: %.2f", trackId, detection.getDetectedObjectClass(), detection.getConfidence())
               : String.format(Locale.US, "%s: %.2f", detection.getDetectedObjectClass(), detection.getConfidence());

         // Draw the bounding box
         Rect boundingBox = new Rect((int) Math.round(detection.getBoundingBox().getMinX()),
                                     (int) Math.round(detection.getBoundingBox().getMinY()),
                                     (int) Math.round(detection.getBoundingBox().getMaxX() - detection.getBoundingBox().getMinX()),
                                     (int) Math.round(detection.getBoundingBox().getMaxY() - detection.getBoundingBox().getMinY()));
         opencv_imgproc.rectangle(annotatedImage, boundingBox, color, 5, LINE_TYPE, 0);

         // Draw text background
         Size textSize = opencv_imgproc.getTextSize(text, FONT, FONT_SCALE, FONT_THICKNESS, new IntPointer());

         int textBoxClampedX = MathTools.clamp(boundingBox.x(), 0, annotatedImage.cols() - textSize.width());
         int textBoxClampedY = MathTools.clamp(boundingBox.y() - textSize.height(), 0, annotatedImage.rows() - textSize.height());

         Rect textBox = new Rect(textBoxClampedX, textBoxClampedY, textSize.width(), textSize.height());
         opencv_imgproc.rectangle(annotatedImage, textBox, color, opencv_imgproc.FILLED, LINE_TYPE, 0);

         // Draw the text
         Point textLocation = new Point(textBoxClampedX, textBoxClampedY + textSize.height());
         opencv_imgproc.putText(annotatedImage, text, textLocation, FONT, FONT_SCALE, WHITE, FONT_THICKNESS, TEXT_LINE_TYPE, false);

         RawImage mask = detection.getObjectMask();
         if (mask != null)
         {
            Mat maskMat = mask.getCpuImageMat();
            if (maskMat != null && !maskMat.isNull())
            {
               Mat resizedMask = new Mat();
               resizeWithCrop(maskMat, resizedMask, annotatedImage.size());

               Mat colorMat = new Mat(annotatedImage.rows(), annotatedImage.cols(), annotatedImage.type(), color);
               opencv_core.add(annotatedImage, colorMat, annotatedImage, resizedMask, -1);

               colorMat.close();
               resizedMask.close();
            }
         }

         boundingBox.close();
         textBox.close();
      }
   }

   public static void annotateTargets(Mat image, List<AnnotatedTarget2D> targets)
   {
      for (AnnotatedTarget2D target : targets)
      {
         if (target.bbox == null)
            continue;

         int x1 = Math.round(target.bbox[0]);
         int y1 = Math.round(target.bbox[1]);
         int x2 = Math.round(target.bbox[2]);
         int y2 = Math.round(target.bbox[3]);

         Scalar color = colorForId(target.targetId);

         if (target.mask != null)
         {
            Mat maskMat = target.mask.getCpuImageMat();
            if (maskMat != null && !maskMat.isNull())
            {
               Mat resizedMask = new Mat();
               resizeWithCrop(maskMat, resizedMask, image.size());

               Mat colorMat = new Mat(image.rows(), image.cols(), image.type(), color);
               opencv_core.add(image, colorMat, image, resizedMask, -1);

               colorMat.close();
               resizedMask.close();
            }
         }

         opencv_imgproc.rectangle(image, new Point(x1, y1), new Point(x2, y2), color, 4, LINE_TYPE, 0);

         String objectName = target.name == null || target.name.isBlank() ? "Object" : prettifyName(target.name);
         String label = String.format(Locale.US, "ID:%d %.2f %s", target.targetId, target.score, objectName);

         Size textSize = opencv_imgproc.getTextSize(label, FONT, FONT_SCALE, FONT_THICKNESS, new IntPointer());

         int textBoxClampedX = MathTools.clamp(x1, 0, image.cols() - textSize.width());
         int textBoxClampedY = MathTools.clamp(y1 - textSize.height(), 0, image.rows() - textSize.height());

         Rect textBox = new Rect(textBoxClampedX, textBoxClampedY, textSize.width(), textSize.height());
         opencv_imgproc.rectangle(image, textBox, color, opencv_imgproc.FILLED, LINE_TYPE, 0);

         Point textLocation = new Point(textBoxClampedX, textBoxClampedY + textSize.height());
         opencv_imgproc.putText(image, label, textLocation, FONT, FONT_SCALE, WHITE, FONT_THICKNESS, TEXT_LINE_TYPE, false);

         textBox.close();
      }
   }

   public static void drawObjectOutlines(Mat inputImage,
                                         Mat annotatedImage,
                                         List<YOLOv8InstantDetection> detections,
                                         Function<YOLOv8InstantDetection, String> textProvider)
   {
      inputImage.copyTo(annotatedImage);

      for (YOLOv8InstantDetection detection : detections)
      {
         BoundingBox2DReadOnly boundingBox = detection.getBoundingBox();

         Point2D boundingBoxCenter = new Point2D();
         detection.getBoundingBox().getCenterPoint(boundingBoxCenter);

         String text = textProvider.apply(detection);
         Size textSize = opencv_imgproc.getTextSize(text, FONT, FONT_SCALE_SMALL, FONT_THICKNESS + 1, new IntPointer());

         int textX = (int) Math.round(boundingBoxCenter.getX()) - textSize.width() / 2;
         int textY;

         if (textSize.width() >= boundingBox.getMaxX() - boundingBox.getMinX())
            textY = (int) Math.round(boundingBox.getMinY()) - textSize.height();
         else
            textY = (int) Math.round(boundingBoxCenter.getY()) - textSize.height() / 2;

         int textBoxClampedX = MathTools.clamp(textX, 0, annotatedImage.cols() - textSize.width());
         int textBoxClampedY = MathTools.clamp(textY, 0, annotatedImage.rows() - textSize.height());

         Point textLocation = new Point(textBoxClampedX, textBoxClampedY);
         opencv_imgproc.putText(annotatedImage, text, textLocation, FONT, FONT_SCALE_SMALL, BLACK, FONT_THICKNESS + 1, TEXT_LINE_TYPE, false);
         opencv_imgproc.putText(annotatedImage, text, textLocation, FONT, FONT_SCALE_SMALL, GREEN, FONT_THICKNESS - 1, TEXT_LINE_TYPE, false);

         RawImage mask = detection.getObjectMask();
         if (mask == null)
            continue;

         Mat maskMat = mask.getCpuImageMat();
         if (maskMat == null || maskMat.isNull())
            continue;

         // Scaling to match annotatedImage and crop away the letter box lines added to the mask during YOLO processing to match aspect ratio
         Mat resizedMask = new Mat();
         resizeWithCrop(maskMat, resizedMask, annotatedImage.size());

         MatVector contours = new MatVector();
         Mat hierarchy = new Mat();
         opencv_imgproc.findContours(resizedMask, contours, hierarchy, opencv_imgproc.RETR_TREE, opencv_imgproc.CHAIN_APPROX_SIMPLE);
         opencv_imgproc.drawContours(annotatedImage, contours, -1, GREEN, 4, LINE_TYPE, hierarchy, Integer.MAX_VALUE, new Point());

         hierarchy.close();
         contours.close();
         resizedMask.close();
      }
   }

   private static String prettifyName(String rawName)
   {
      if (rawName == null || rawName.isBlank())
         return "Object";

      String cleaned = rawName.replace('_', ' ').trim().toLowerCase(Locale.US);
      if (cleaned.isEmpty())
         return "Object";

      return Character.toUpperCase(cleaned.charAt(0)) + cleaned.substring(1);
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

      directories.sort(Comparator.comparing(URL::toString));

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