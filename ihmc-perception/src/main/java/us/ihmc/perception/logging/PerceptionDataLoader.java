package us.ihmc.perception.logging;

import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoOpenCVTools;

import java.io.File;
import java.nio.FloatBuffer;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKeyEx;

public class PerceptionDataLoader
{
   private HashMap<String, PerceptionLogChannel> channels;

   private HDF5Manager hdf5Manager;
   private String filePath;

   public PerceptionDataLoader()
   {
      channels = new HashMap<>();
   }

   public void openLogFile(String filePath)
   {
      LogTools.info("Loading Perception Log: {}", filePath);
      if (Files.exists(Paths.get(filePath)))
      {
         this.filePath = filePath;
         hdf5Manager = new HDF5Manager(filePath, hdf5.H5F_ACC_RDONLY);
         channels.clear();

         ArrayList<String> topicNames = HDF5Tools.getTopicNames(hdf5Manager.getFile());
         for (String topic : topicNames)
         {
            channels.put(topic, new PerceptionLogChannel(topic, (int) hdf5Manager.getCount(topic), 0));
         }
      }
      else
      {
         LogTools.warn("Log file does not exist: {}", filePath);
      }
   }

   public void loadPointCloud(String namespace, int index, RecyclingArrayList<Point3D32> points, int rows, int cols)
   {
      HDF5Tools.loadPointCloud(hdf5Manager.getGroup(namespace), index, points, rows, cols);
   }

   public FloatBuffer loadCompressedPointCloud(String namespace, int index)
   {
      Group group = hdf5Manager.getGroup(namespace);
      byte[] compressedByteArray = HDF5Tools.loadByteArray(group, index);

      LogTools.info("Byte Array: {} {} {}", compressedByteArray[0], compressedByteArray[1], compressedByteArray[2]);

      return null;
   }

   public void loadPoint3DList(String namespace, int index, ArrayList<Point3D> points)
   {
      Group group = hdf5Manager.getGroup(namespace);
      float[] pointFloatArray = HDF5Tools.loadFloatArray(group, index);

      for (int i = 0; i < pointFloatArray.length / 3; i++)
      {
         points.add(new Point3D(pointFloatArray[i], pointFloatArray[i * 3 + 1], pointFloatArray[i * 3 + 2]));

         LogTools.info("Point: {} {} {}", pointFloatArray[i], pointFloatArray[i * 3 + 1], pointFloatArray[i * 3 + 2]);
      }
   }

   public void loadCompressedImage(String namespace, int index, Mat mat)
   {
      Group group = hdf5Manager.getGroup(namespace);
      byte[] compressedByteArray = HDF5Tools.loadByteArray(group, index);

      mat.put(BytedecoOpenCVTools.decompressImageJPGUsingYUV(compressedByteArray));

   }

   public void loadCompressedDepth(String namespace, int index, Mat mat)
   {
      Group group = hdf5Manager.getGroup(namespace);
      byte[] compressedByteArray = HDF5Tools.loadByteArray(group, index);
      BytedecoOpenCVTools.decompressDepthPNG(compressedByteArray, mat);
   }

   public String getFilePath()
   {
      return filePath;
   }

   public HashMap<String, PerceptionLogChannel> getChannels()
   {
      return channels;
   }

   public HDF5Manager getHDF5Manager()
   {
      return hdf5Manager;
   }

   public void destroy()
   {
      hdf5Manager.closeFile();
   }

   public static void main(String[] args)
   {
      String defaultLogDirectory =
            System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator + "perception" + File.separator;
      String LOG_DIRECTORY = System.getProperty("perception.log.directory", defaultLogDirectory);
      String logFileName = "20230113_161357_Duncan_Images.hdf5";

      PerceptionDataLoader loader = new PerceptionDataLoader();
      loader.openLogFile(LOG_DIRECTORY + logFileName);

      ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();

      long total = loader.getHDF5Manager().getCount("/image/");
//      long total = loader.getHDF5Manager().getCount(PerceptionLoggerConstants.OUSTER_DEPTH_NAME);

      //      long total = Math.min(totalColor, totalDepth);

      Mat colorImage = new Mat();
      Mat depthImage = new Mat(128, 2048, opencv_core.CV_16UC1);
      //      LogTools.info("Total Images: {}", totalDepth);

      ArrayList<Point3D> points = new ArrayList<>();

      for (int i = 0; i < total; i++)
      {
         //         points.clear();
         //         loader.loadPoint3DList(PerceptionLoggerConstants.L515_SENSOR_POSITION, i, points);

         LogTools.info("Loading Index: {}/{}", i, 10);
         //         loader.loadCompressedImage(PerceptionLoggerConstants.L515_COLOR_NAME, i, colorImage);

         long begin_load = System.nanoTime();
         loader.loadCompressedImage("/image/", i, colorImage);
//         loader.loadCompressedDepth("/image/", i, colorImage);
         long end_load = System.nanoTime();

//         LogTools.info("Depth Image Format: {} {}", BytedecoOpenCVTools.getTypeString(depthImage.type()), depthImage.channels());
//
//         long begin_decompress = System.nanoTime();
//         Mat displayDepth = new Mat(depthImage.rows(), depthImage.cols(), opencv_core.CV_8UC1);
//         Mat finalDisplayDepth = new Mat(depthImage.rows(), depthImage.cols(), opencv_core.CV_8UC3);
//
//         BytedecoOpenCVTools.clampTo8BitUnsignedChar(depthImage, displayDepth, 0.0, 255.0);
//         BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(displayDepth, finalDisplayDepth);
         //         long end_decompress = System.nanoTime();

         //         LogTools.info("Loading Time: {} ms", (end_load - begin_load) / 1e6);
         //         LogTools.info("Decompression Time: {} ms", (end_decompress - begin_decompress) / 1e6f);

                  imshow("/l515/color", colorImage);
//         imshow(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, finalDisplayDepth);
         int code = waitKeyEx(100);
         if (code == 113)
         {
            System.exit(0);
         }
      }
   }
}
