package us.ihmc.perception.tools;

import gnu.trove.list.array.TIntArrayList;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_highgui;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

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

   public static void printMat(String name, Mat image)
   {
      LogTools.info(matToString(name, image));
   }

   public static String matToString(String name, Mat image)
   {
      StringBuilder matString = new StringBuilder("Mat: [" + name + "]\n");

      for (int i = 0; i < image.rows(); i++)
      {
         for (int j = 0; j < image.cols(); j++)
         {
            if (image.type() == opencv_core.CV_16UC1)
               matString.append(image.ptr(i, j).getShort()).append("\t");
            if (image.type() == opencv_core.CV_64FC1)
               matString.append("%.5f\t".formatted(image.ptr(i, j).getDouble()));
         }
         matString.append("\n");
      }

      return matString.toString();
   }

   public static void display(String tag, Mat image, int delay)
   {
      opencv_highgui.imshow(tag, image);
      int code = opencv_highgui.waitKeyEx(delay);
      if (code == 113) // Keycode for 'q'
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
      BytedecoOpenCVTools.clampTo8BitUnsignedChar(image, displayDepth, 0.0, 250.0);
      BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(displayDepth, finalDisplayDepth);

      opencv_imgproc.resize(finalDisplayDepth, finalDisplayDepth, new Size((int) (image.cols() * scale), (int) (image.rows() * scale)));
      display(tag, finalDisplayDepth, delay);
   }

   public static void displayHeightMap(String tag, Mat image, int delay, float scale)
   {
      Mat displayDepth = new Mat(image.rows(), image.cols(), opencv_core.CV_8UC1);
      Mat finalDisplayDepth = new Mat(image.rows(), image.cols(), opencv_core.CV_8UC3);

      BytedecoOpenCVTools.clampTo8BitUnsignedChar(image, displayDepth, 0.0, 250.0);

      opencv_imgproc.threshold(displayDepth, displayDepth, 100, 255, opencv_imgproc.CV_THRESH_TOZERO_INV);
      opencv_core.normalize(displayDepth, displayDepth, 255, 0, opencv_core.NORM_MINMAX, opencv_core.CV_8UC1, new Mat());

      BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(displayDepth, finalDisplayDepth);

      opencv_imgproc.resize(finalDisplayDepth, finalDisplayDepth, new Size((int) (image.cols() * scale), (int) (image.rows() * scale)));
      display(tag, finalDisplayDepth, delay);
   }
}
