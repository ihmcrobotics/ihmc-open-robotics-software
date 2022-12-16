package us.ihmc.perception.logging;

import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.io.File;
import java.nio.FloatBuffer;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKeyEx;

public class PerceptionDataLoader
{
   private ArrayList<PerceptionLogChannel> channels;

   private HDF5Manager hdf5Manager;
   private String filePath;

   public PerceptionDataLoader()
   {
      channels = new ArrayList<>();
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
            channels.add(new PerceptionLogChannel(topic, (int) hdf5Manager.getCount(topic), 0));
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

   public void loadImage(String namespace, int index, Mat mat)
   {
      //      ThreadTools.startAThread(()->{
      LogTools.info("Loading Image: {} {}", namespace, index);

      Group group = hdf5Manager.getGroup(namespace);
      byte[] compressedByteArray = HDF5Tools.loadByteArray(group, index);

      mat.put(BytedecoOpenCVTools.decompressImageJPGUsingYUV(compressedByteArray));

      LogTools.info("Completed Loading Image: {} {} {}", index, compressedByteArray.length);
      //      }, "perception_data_loader -> " + namespace);
   }

   public void loadDepth(String namespace, int index, Mat mat)
   {
      LogTools.info("Loading Image: {} {}", namespace, index);

      Group group = hdf5Manager.getGroup(namespace);
      byte[] compressedByteArray = HDF5Tools.loadByteArray(group, index);

      //      LogTools.info("Depth: {}", Arrays.toString(compressedByteArray));

      BytedecoOpenCVTools.decompressDepthPNG(compressedByteArray, mat);

      LogTools.info("Completed Loading Image: {} {}", index, compressedByteArray.length);
   }

   public String getFilePath()
   {
      return filePath;
   }

   public ArrayList<PerceptionLogChannel> getChannels()
   {
      return channels;
   }

   public HDF5Manager getHDF5Manager()
   {
      return hdf5Manager;
   }

   public static void main(String[] args)
   {
      String defaultLogDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator + "perception" + File.separator;
      String LOG_DIRECTORY = System.getProperty("perception.log.directory", defaultLogDirectory);
      String logFileName = "20221215_234308_PerceptionLog.hdf5";

      PerceptionDataLoader loader = new PerceptionDataLoader();
      loader.openLogFile(LOG_DIRECTORY + logFileName);

      ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();

      long totalColor = loader.getHDF5Manager().getCount("/l515/color/");
      long totalDepth = loader.getHDF5Manager().getCount("/l515/depth/");

      long total = Math.min(totalColor, totalDepth);

      Mat colorImage = new Mat();
      Mat depthImage = new Mat(720, 1280, opencv_core.CV_16UC1);
      LogTools.info("Total Images: {}", totalColor);

      for (int i = 0; i < total; i++)
      {

         LogTools.info("Loading Index: {}/{}", i, total);
         loader.loadImage("/l515/color/", i, colorImage);


         long begin_load = System.nanoTime();
         loader.loadDepth("/l515/depth/", i, depthImage);
         long end_load = System.nanoTime();

         LogTools.info("Depth Image Format: {} {}", BytedecoOpenCVTools.getTypeString(depthImage.type()), depthImage.channels());

         long begin_decompress = System.nanoTime();
         Mat displayDepth = new Mat(depthImage.rows(), depthImage.cols(), opencv_core.CV_8UC1);
         Mat finalDisplayDepth = new Mat(depthImage.rows(), depthImage.cols(), opencv_core.CV_8UC3);

         BytedecoOpenCVTools.clampTo8BitUnsignedChar(depthImage, displayDepth, 0.0, 255.0);
         BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(displayDepth, finalDisplayDepth);
         long end_decompress = System.nanoTime();

         LogTools.info("Loading Time: {} ms", (end_load - begin_load) / 1e6);
         LogTools.info("Decompression Time: {} ms", (end_decompress - begin_decompress) / 1e6f);

         imshow("/l515/color", colorImage);
         imshow("/l515/depth", finalDisplayDepth);
         int code = waitKeyEx(30);
         if (code == 113)
         {
            System.exit(0);
         }
      }


   }
}
