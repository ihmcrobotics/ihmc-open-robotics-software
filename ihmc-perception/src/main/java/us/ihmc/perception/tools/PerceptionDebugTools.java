package us.ihmc.perception.tools;

import gnu.trove.list.array.TIntArrayList;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.PointerPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_highgui;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.*;
import org.ejml.data.FMatrixRMaj;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint3f;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.nio.FloatBuffer;
import java.util.Arrays;
import java.util.HashMap;

public class PerceptionDebugTools
{
   public static final Scalar COLOR_RED = new Scalar(0, 0, 255, 255);
   public static final Scalar COLOR_GREEN = new Scalar(0, 255, 0, 255);
   public static final Scalar COLOR_BLUE = new Scalar(255, 0, 0, 255);
   public static final Scalar COLOR_YELLOW = new Scalar(0, 255, 255, 255);
   public static final Scalar COLOR_BLACK = new Scalar(0, 0, 0, 255);
   public static final Scalar COLOR_WHITE = new Scalar(255, 255, 255, 255);
   public static final Scalar COLOR_GRAY = new Scalar(128, 128, 128, 255);
   public static final Scalar COLOR_PURPLE = new Scalar(255, 0, 160, 255);

   public static void printMatches(String tag, PlanarRegionsList map, PlanarRegionsList regions, HashMap<Integer, TIntArrayList> matches, boolean debug)
   {
      if (!debug)
         return;

      LogTools.info("------------------------------------------------ Printing Matches ({}) ---------------------------------------------", tag);
      LogTools.info("Map Region Count: {}", map.getNumberOfPlanarRegions());
      LogTools.info("Incoming Regions Count: {}", regions.getNumberOfPlanarRegions());

      for (Integer key : matches.keySet())
      {
         int[] values = matches.get(key).toArray();
         LogTools.info("Match: ({}) -> {}", key, Arrays.toString(values));
      }
      LogTools.info("------------------------------------------------ Printing Matches End ---------------------------------------------");
   }

   public static void printPlanarRegionsListVertices(String tag, PlanarRegionsList regions, boolean debug)
   {
      if (!debug)
         return;

      LogTools.info("[{}]", tag);
      for (int i = 0; i < regions.getNumberOfPlanarRegions(); i++)
      {
         LogTools.info("Region Index: {}, Region ID: {}", i, regions.getPlanarRegion(i).getRegionId());
         printPlanarRegionVertices(regions.getPlanarRegion(i), debug);
      }
   }

   public static void printPlanarRegionVertices(PlanarRegion region, boolean debug)
   {
      if (!debug)
         return;

      LogTools.info("Concave Hull Vertices -----------------------------------");
      for (int i = 0; i < region.getConcaveHullSize(); i++)
      {
         LogTools.info("[Region: {}] Point {} : {}", region.getRegionId(), i, region.getConcaveHullPoint3DInWorld(i));
      }
      System.out.println();
   }

   public static void printRegionIDs(String tag, PlanarRegionsList regions, boolean debug)
   {
      if (!debug)
         return;

      int[] ids = new int[regions.getNumberOfPlanarRegions()];
      for (int i = 0; i < regions.getNumberOfPlanarRegions(); i++)
      {
         ids[i] = regions.getPlanarRegion(i).getRegionId();
      }
      LogTools.info("[{}] Region IDs: {}", tag, Arrays.toString(ids));
   }

   public static void printPlane(PlanarRegion region, boolean debug)
   {
      if (!debug)
         return;

      LogTools.info("Plane: {}",
                    String.format("%.2f, %.2f, %.2f, %.2f",
                                  region.getNormal().getX(),
                                  region.getNormal().getY(),
                                  region.getNormal().getZ(),
                                  region.getNormal().dot(region.getPoint())));
   }

   public static void printTransform(String tag, RigidBodyTransform transform, boolean debug)
   {
      if (!debug)
         return;

      Point3D euler = new Point3D();
      transform.getRotation().getEuler(euler);

      LogTools.info("[{}] Translation: {}, Rotation: {}", tag, transform.getTranslation(), euler);
   }

   public static String getTypeString(int type)
   {
      int numImgTypes = 35; // 7 base types, with five channel options each (none or C1, ..., C4)

      int enum_ints[] = {opencv_core.CV_8U,
                         opencv_core.CV_8UC1,
                         opencv_core.CV_8UC2,
                         opencv_core.CV_8UC3,
                         opencv_core.CV_8UC4,
                         opencv_core.CV_8S,
                         opencv_core.CV_8SC1,
                         opencv_core.CV_8SC2,
                         opencv_core.CV_8SC3,
                         opencv_core.CV_8SC4,
                         opencv_core.CV_16U,
                         opencv_core.CV_16UC1,
                         opencv_core.CV_16UC2,
                         opencv_core.CV_16UC3,
                         opencv_core.CV_16UC4,
                         opencv_core.CV_16S,
                         opencv_core.CV_16SC1,
                         opencv_core.CV_16SC2,
                         opencv_core.CV_16SC3,
                         opencv_core.CV_16SC4,
                         opencv_core.CV_32S,
                         opencv_core.CV_32SC1,
                         opencv_core.CV_32SC2,
                         opencv_core.CV_32SC3,
                         opencv_core.CV_32SC4,
                         opencv_core.CV_32F,
                         opencv_core.CV_32FC1,
                         opencv_core.CV_32FC2,
                         opencv_core.CV_32FC3,
                         opencv_core.CV_32FC4,
                         opencv_core.CV_64F,
                         opencv_core.CV_64FC1,
                         opencv_core.CV_64FC2,
                         opencv_core.CV_64FC3,
                         opencv_core.CV_64FC4};

      String enum_strings[] = {"CV_8U",
                               "CV_8UC1",
                               "CV_8UC2",
                               "CV_8UC3",
                               "CV_8UC4",
                               "CV_8S",
                               "CV_8SC1",
                               "CV_8SC2",
                               "CV_8SC3",
                               "CV_8SC4",
                               "CV_16U",
                               "CV_16UC1",
                               "CV_16UC2",
                               "CV_16UC3",
                               "CV_16UC4",
                               "CV_16S",
                               "CV_16SC1",
                               "CV_16SC2",
                               "CV_16SC3",
                               "CV_16SC4",
                               "CV_32S",
                               "CV_32SC1",
                               "CV_32SC2",
                               "CV_32SC3",
                               "CV_32SC4",
                               "CV_32F",
                               "CV_32FC1",
                               "CV_32FC2",
                               "CV_32FC3",
                               "CV_32FC4",
                               "CV_64F",
                               "CV_64FC1",
                               "CV_64FC2",
                               "CV_64FC3",
                               "CV_64FC4"};

      for (int i = 0; i < numImgTypes; i++)
      {
         if (type == enum_ints[i])
            return enum_strings[i];
      }
      return "unknown image type";
   }

   public static void printBuffer2D(String tag, FloatBuffer buffer, int rows, int cols, int skip)
   {
      LogTools.info(bufferToString(tag, buffer, rows, cols, skip));
   }

   public static String bufferToString(String tag, FloatBuffer buffer, int rows, int cols, int skip)
   {
      StringBuilder matString = new StringBuilder("Buffer: [" + tag + "]\n");
      for (int i = 0; i < rows; i += skip)
      {
         for (int j = 0; j < cols; j += skip)
         {
            matString.append(String.format("%.2f ", buffer.get(i * cols + j)));
         }
         matString.append("\n");
      }
      return matString.toString();
   }

   public static void printMat(String name, Mat image, int skip)
   {
      LogTools.info(matToString(name, image, 0, 0, image.rows(), image.cols(), skip));
   }

   public static void printMat(String name, Mat image, int rowBegin, int colBegin, int rowEnd, int colEnd, int skip)
   {
      LogTools.info(matToString(name, image, rowBegin, colBegin, rowEnd, colEnd, skip));
   }

   public static void printMatVector(String name, MatVector matVector)
   {
      for (int i = 0; i < matVector.size(); i++)
      {
         printMat("%s %d:".formatted(name, i), matVector.get(i), 1);
         System.out.println();
      }
   }

   public static void printHeightMap(String name, HeightMapData heightMapData, int skip)
   {
      LogTools.info(heightMapToString(name, heightMapData, skip));
   }

   public static String heightMapToString(String name, HeightMapData heightMapData, int skip)
   {
      StringBuilder matString = new StringBuilder("Mat: [" + name + "]\n");
      LogTools.info("Height Map: [Center: {}, Size: {}]", heightMapData.getGridCenter(), heightMapData.getCellsPerAxis());
      for (int i = 0; i < heightMapData.getCellsPerAxis(); i += skip)
      {
         for (int j = 0; j < heightMapData.getCellsPerAxis(); j += skip)
         {
            double height = heightMapData.getHeightAt(i, j);
            //if (height > 0.0001)
               matString.append(String.format("%.1f", height)).append(" ");
            //else
            //   matString.append("||||").append(" ");
         }
         matString.append("\n");
      }
      return matString.toString();
   }

   public static String matToString(String name, Mat image, int rowBegin, int colBegin, int rowEnd, int colEnd, int skip)
   {
      StringBuilder matString = new StringBuilder("Mat: [" + name + "], Type: [" + getTypeString(image.type()) + "], Channels: [" + image.channels() + "], ");
      matString.append("Dims: [").append(image.rows()).append(", ").append(image.cols()).append("], ");
      matString.append("Crop: [").append(rowBegin).append(", ").append(colBegin).append(", ").append(rowEnd).append(", ").append(colEnd).append("]\n");

      for (int i = rowBegin; i < rowEnd; i += skip)
      {
         for (int j = colBegin; j < colEnd; j += skip)
         {
            if (image.type() == opencv_core.CV_8UC1)
               matString.append(image.ptr(i, j).get() & 0xFF).append(" ");
            if (image.type() == opencv_core.CV_16UC1)
               matString.append(((int) image.ptr(i, j).getShort()) & 0xFFFF).append("\t");
            if (image.type() == opencv_core.CV_64FC1)
               matString.append("%.5f\t".formatted(image.ptr(i, j).getDouble()));
            if (image.type() == opencv_core.CV_32FC1)
               matString.append("%.2f\t".formatted(image.ptr(i, j).getFloat()));
            if (image.type() == opencv_core.CV_32FC2)
               matString.append("%.5f\t%.5f\t\t".formatted(image.ptr(i, j).getFloat(), image.ptr(i, j).getFloat(Float.BYTES)));
         }
         matString.append("\n");
      }

      return matString.toString();
   }

   public static void display(String tag, Mat image, int delay)
   {
      display(tag, image, delay, -1);
   }

   public static void display(String tag, Mat image, int delay, int screenSize)
   {
      opencv_highgui.namedWindow(tag, opencv_highgui.WINDOW_NORMAL);

      if (screenSize != -1)
      {
         int finalRows = screenSize;
         int finalCols = (int)((float) screenSize / (float) image.rows() * (float) image.cols());
         LogTools.debug(String.format("Image Size: %d x %d Display Size: %d x %d", finalRows, finalCols, image.rows(), image.cols()));
         opencv_highgui.resizeWindow(tag, finalCols, finalRows);
      }

      opencv_highgui.imshow(tag, image);
      int code = opencv_highgui.waitKeyEx(delay);
      if (code == 113) // Keycode for 'q'
      {
         System.exit(0);
      }
   }

   public static void displayDepth(String tag, Mat image, int delay)
   {
      displayDepth(tag, image, delay, 1000);
   }

   public static void displayDepth(String tag, Mat image, int delay, int screenSize)
   {
      Mat displayDepth = new Mat(image.rows(), image.cols(), opencv_core.CV_8UC1);
      Mat finalDisplayDepth = new Mat(image.rows(), image.cols(), opencv_core.CV_8UC3);

      image.convertTo(displayDepth, opencv_core.CV_8UC1, 0.1, 0);
      OpenCVTools.convert8BitGrayTo8BitRGBA(displayDepth, finalDisplayDepth);

      display(tag, finalDisplayDepth, delay, screenSize);
   }

   public static void displayHeightMap(String tag, Mat image, int delay, float scale)
   {
      Mat displayDepth = new Mat(image.rows(), image.cols(), opencv_core.CV_8UC1);
      Mat finalDisplayDepth = new Mat(image.rows(), image.cols(), opencv_core.CV_8UC3);

      OpenCVTools.clampTo8BitUnsignedChar(image, displayDepth, 0.0, 250.0);

      opencv_imgproc.threshold(displayDepth, displayDepth, 100, 255, opencv_imgproc.CV_THRESH_TOZERO_INV);
      opencv_core.normalize(displayDepth, displayDepth, 255, 0, opencv_core.NORM_MINMAX, opencv_core.CV_8UC1, new Mat());

      OpenCVTools.convert8BitGrayTo8BitRGBA(displayDepth, finalDisplayDepth);

      opencv_imgproc.resize(finalDisplayDepth, finalDisplayDepth, new Size((int) (image.cols() * scale), (int) (image.rows() * scale)));
      display(tag, finalDisplayDepth, delay, finalDisplayDepth.rows());
   }

   public void testProjection(Mat depth)
   {
      double radius = 4.0f;
      double height = 2.0f;
      double yawUnit = 2 * Math.PI / depth.cols();
      double pitchUnit = Math.PI / (2 * depth.rows());

      for (int i = 0; i < depth.cols(); i++)
      {
         Point3D point = new Point3D(radius * Math.cos(i * yawUnit), radius * Math.sin(i * yawUnit), -height);

         Point2D projection = sphericalProject(point, depth.rows(), depth.cols());

         LogTools.info("[" + i + "] Point : " + String.format("%.2f, %.2f, %.2f", point.getX(), point.getY(), point.getZ()) + " Projection : " + String.format(
               "%d, %d",
               (int) projection.getX(),
               (int) projection.getY()));
      }
   }

   public Point2D sphericalProject(Point3D cellCenter, int INPUT_HEIGHT, int INPUT_WIDTH)
   {
      Point2D proj = new Point2D();

      double pitchUnit = Math.PI / (2 * INPUT_HEIGHT);
      double yawUnit = 2 * Math.PI / (INPUT_WIDTH);

      int pitchOffset = INPUT_HEIGHT / 2;
      int yawOffset = INPUT_WIDTH / 2;

      double x = cellCenter.getX();
      double y = cellCenter.getY();
      double z = cellCenter.getZ();

      double radius = Math.sqrt(x * x + y * y);

      double pitch = Math.atan2(z, radius);
      int pitchCount = (pitchOffset) - (int) (pitch / pitchUnit);

      double yaw = Math.atan2(-y, x);
      int yawCount = (yawOffset) + (int) (yaw / yawUnit);

      proj.setX(pitchCount);
      proj.setY(yawCount);

      LogTools.info(String.format("Projection: [%.2f,%.2f] (Yc:%d,Pc:%d, Z:%.2f,R:%.2f)\n", yaw, pitch, yawCount, pitchCount, z, radius));

      return proj;
   }

   public static void plotFootsteps(Mat displayImage, FMatrixRMaj linearOutput, int size)
   {
      Scalar leftColor = new Scalar(0, 255, 255, 0);
      Scalar rightColor = new Scalar(0, 0, 255, 0);

      for (int i = 0; i < linearOutput.getNumElements() / 2; i++)
      {
         Point2D point = new Point2D(linearOutput.get(2 * i, 0), linearOutput.get(2 * i + 1, 0));
         Point2D positionOnMap = new Point2D(point.getY() * 50 + displayImage.rows() / 2, point.getX() * 50 + displayImage.cols() / 2);
         opencv_imgproc.rectangle(displayImage,
                                  new Point((int) positionOnMap.getX() - size, (int) positionOnMap.getY() - size),
                                  new Point((int) positionOnMap.getX() + size, (int) positionOnMap.getY() + size),
                                  i % 2 == 1 ? leftColor : rightColor,
                                  -1,
                                  opencv_imgproc.LINE_4,
                                  0);
      }
   }

   public static void plotRectangle(Mat displayImage, Point2D point, int size, Scalar color)
   {
      LogTools.info("Plotting Node: Footstep: {} {}", (int) (point.getY() * 50 + displayImage.rows() / 2), (int) (point.getX() * 50 + displayImage.cols() / 2));

      // just like plotFootsteps
      Point2D positionOnMap = new Point2D(point.getY() * 50 + displayImage.rows() / 2, point.getX() * 50 + displayImage.cols() / 2);
      opencv_imgproc.rectangle(displayImage,
                               new Point((int) positionOnMap.getX() - size, (int) positionOnMap.getY() - size),
                               new Point((int) positionOnMap.getX() + size, (int) positionOnMap.getY() + size),
                               color,
                               -1,
                               opencv_imgproc.LINE_4,
                               0);
   }

   public static void plotRectangleNoScale(Mat displayImage, Point2D point, int size, Scalar color)
   {
      LogTools.debug("Plotting Node: Footstep: {} {}", (int) (point.getY() + displayImage.rows() / 2), (int) (point.getX() + displayImage.cols() / 2));

      // just like plotFootsteps
      Point2D positionOnMap = new Point2D(point.getY() + displayImage.rows() / 2, point.getX() + displayImage.cols() / 2);
      opencv_imgproc.rectangle(displayImage,
                               new Point((int) positionOnMap.getX() - size, (int) positionOnMap.getY() - size),
                               new Point((int) positionOnMap.getX() + size, (int) positionOnMap.getY() + size),
                               color,
                               -1,
                               opencv_imgproc.LINE_4,
                               0);
   }

   public static void plotTiltedRectangle(Mat displayImage, Point2D origin, float yaw, int size, int side)
   {
      LogTools.debug("Footstep Plotted: {} {} {}",
                     (int) (origin.getY() * 50 + displayImage.rows() / 2),
                     (int) (origin.getX() * 50 + displayImage.cols() / 2),
                     side);

      Scalar color;

      switch (side)
      {
         case 1: // right foot is red
            color = new Scalar(0, 100, 255, 0);
            break;
         case -1: // left foot is blue
            color = new Scalar(255, 100, 0, 0);
            break;
         case 2: // start poses are black
            color = new Scalar(0, 255, 255, 0);
            break;
         case 3: // goal poses are white
            color = new Scalar(255, 255, 255, 0);
            break;
         default:
            color = new Scalar(255, 255, 255, 255);
            break;
      }

      // just like plotFootsteps
      //Point2D positionOnMap = new Point2D(origin.getY() * 50 + displayImage.rows() / 2, origin.getX() * 50 + displayImage.cols() / 2);
      //opencv_imgproc.rectangle(displayImage,
      //                         new Point((int) positionOnMap.getX() - size, (int) positionOnMap.getY() - size * 2),
      //                         new Point((int) positionOnMap.getX() + size, (int) positionOnMap.getY() + size * 2),
      //                         color,
      //                         -1,
      //                         opencv_imgproc.LINE_4,
      //                         0);

      plotFootstepWithYaw(displayImage, new Point3D(origin.getX(), origin.getY(), yaw), color, -1, size, size * 2);
   }

   public static void plotFootstepWithYaw(Mat display, Point3D imageCoordinatesWithYaw, Scalar color, int index, double width, double length)
   {
      double[] positionOnMap = {imageCoordinatesWithYaw.getX() + display.rows() / 2, imageCoordinatesWithYaw.getY() + display.cols() / 2};
      double yaw = imageCoordinatesWithYaw.getZ();

      // Create the footstep rectangle using the position and orientation
      Point3D[] points = {new Point3D(-length, -width, 0),
                          new Point3D(-length, width, 0),
                          new Point3D(length, width, 0),
                          new Point3D(length, -width, 0)};

      Quaternion quat = new Quaternion();
      quat.setYawPitchRoll(yaw + Math.PI / 2, 0, 0);

      Vector3D translation = new Vector3D(positionOnMap[1], positionOnMap[0], 0);

      RigidBodyTransform transform = new RigidBodyTransform(quat, translation);

      // Rotate the points
      for (Point3D point : points)
      {
         point.applyTransform(transform);
      }

      Mat matOfPoints = new Mat(4, 2, opencv_core.CV_32SC1);

      for (int i = 0; i < 4; i++)
      {
         matOfPoints.ptr(i, 0).putInt((int) points[i].getX());
         matOfPoints.ptr(i, 1).putInt((int) points[i].getY());
      }

      // Draw the polyline on the image
      opencv_imgproc.fillConvexPoly(display, matOfPoints, color);

      // Plot text on the footstep which is the index of the footstep (size 0.1)
      if (index != -1)
      {
         String text = String.valueOf(index);
         Size textsize = opencv_imgproc.getTextSize(text, opencv_imgproc.FONT_HERSHEY_SIMPLEX, 0.1, 1, new IntPointer());
         int textX = (int) (positionOnMap[0] - textsize.width() / 2) + 2;
         int textY = (int) (positionOnMap[1] + textsize.height() / 2);
         opencv_imgproc.putText(display, text, new Point(textX, textY), opencv_imgproc.FONT_HERSHEY_SIMPLEX, 0.1, color, 1, 0, false);
      }
   }

   public static void plotCircle(Mat displayImage, Point2D origin, int radius)
   {
      Scalar color = new Scalar(0, 255, 255, 255);

      Point2D positionOnMap = new Point2D(origin.getY() * 50 + displayImage.rows() / 2, origin.getX() * 50 + displayImage.cols() / 2);
      opencv_imgproc.circle(displayImage, new Point((int) positionOnMap.getX(), (int) positionOnMap.getY()), radius, color, -1, opencv_imgproc.LINE_4, 0);
   }

   public static void convertDepthCopyToColor(Mat depthImage16UC1Copy, Mat colorImage8UC3)
   {
      opencv_core.convertScaleAbs(depthImage16UC1Copy, depthImage16UC1Copy, 255.0 / 65535.0, 0);
      opencv_imgproc.cvtColor(depthImage16UC1Copy, colorImage8UC3, opencv_imgproc.COLOR_GRAY2RGB);
   }

   public static void clearAllWindows()
   {
      opencv_highgui.destroyAllWindows();
   }
}
