package us.ihmc.perception.tools;

import gnu.trove.list.array.TIntArrayList;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_highgui;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.Arrays;
import java.util.HashMap;

public class PerceptionDebugTools
{
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
      for(int i = 0; i < regions.getNumberOfPlanarRegions(); i++)
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

   public static void printMat(String name, Mat image, int skip)
   {
      LogTools.info(matToString(name, image, skip));
   }

   public static void printMatVector(String name, MatVector matVector)
   {
      for (int i = 0; i < matVector.size(); i++)
      {
         LogTools.info(matToString("%s %d:".formatted(name, i), matVector.get(i), 1) + "\n");
      }
   }

   public static void printHeightMap(String name, HeightMapData heightMapData, int skip)
   {
      LogTools.info(heightMapToString(name, heightMapData, skip));
   }

   public static String heightMapToString(String name, HeightMapData heightMapData, int skip)
   {
      StringBuilder matString = new StringBuilder("Mat: [" + name + "]\n");
      LogTools.info("Height Map: [Center: {}]", heightMapData.getGridCenter());
      for (int i = 0; i<heightMapData.getCellsPerAxis(); i+=skip)
      {
         for (int j = 0; j<heightMapData.getCellsPerAxis(); j+=skip)
         {
            double height = heightMapData.getHeightAt(i, j);
            if (height > 0.0001)
               matString.append(String.format("%.2f", height)).append(" ");
            else
               matString.append("||||").append(" ");
         }
         matString.append("\n");
      }
      return matString.toString();
   }

   public static String matToString(String name, Mat image, int skip)
   {
      StringBuilder matString = new StringBuilder("Mat: [" + name + "]\n");

      for (int i = 0; i < image.rows(); i+=skip)
      {
         for (int j = 0; j < image.cols(); j+=skip)
         {
            if (image.type() == opencv_core.CV_16UC1)
               matString.append(image.ptr(i, j).getShort()).append("\t");
            if (image.type() == opencv_core.CV_64FC1)
               matString.append("%.5f\t".formatted(image.ptr(i, j).getDouble()));
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
      opencv_highgui.imshow(tag, image);

      if (screenSize != -1)
      {
         opencv_highgui.resizeWindow(tag, screenSize, screenSize);
      }

      int code = opencv_highgui.waitKeyEx(delay);
      if (code == 113 || code != -1) // Keycode for 'q'
      {
         System.exit(0);
      }
   }

   public static void displayDepth(String tag, Mat image, int delay)
   {
      displayDepth(tag, image, delay, 1.0f);
   }

   public static void displayDepth(String tag, Mat image, int delay, float scale)
   {
      Mat displayDepth = new Mat(image.rows(), image.cols(), opencv_core.CV_8UC1);
      Mat finalDisplayDepth = new Mat(image.rows(), image.cols(), opencv_core.CV_8UC3);

      displayDepth.convertTo(displayDepth, opencv_core.CV_8UC1, 0.8, 50);
      OpenCVTools.clampTo8BitUnsignedChar(image, displayDepth, 0.0, 250.0);
      OpenCVTools.convert8BitGrayTo8BitRGBA(displayDepth, finalDisplayDepth);

      opencv_imgproc.resize(finalDisplayDepth, finalDisplayDepth, new Size((int) (image.cols() * scale), (int) (image.rows() * scale)));
      display(tag, finalDisplayDepth, delay);
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
      display(tag, finalDisplayDepth, delay);
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

   public static void clearAllWindows()
   {
      opencv_highgui.destroyAllWindows();
   }
}
