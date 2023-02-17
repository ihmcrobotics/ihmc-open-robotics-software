package us.ihmc.perception.logging;

import org.bytedeco.hdf5.DataSet;
import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_highgui;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.ScheduledExecutorService;

public class PerceptionDataLoader
{
   private HashMap<String, PerceptionLogChannel> channels;

   private final HDF5Tools hdf5Tools = new HDF5Tools();
   private HDF5Manager hdf5Manager;
   private String filePath;

   public PerceptionDataLoader()
   {
      channels = new HashMap<>();
   }

   private final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                        getClass(),
                                                                                                        ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   public void openLogFile(String filePath)
   {
      LogTools.info("Loading Perception Log: {}", filePath);
      if (Files.exists(Paths.get(filePath)))
      {
         this.filePath = filePath;
         hdf5Manager = new HDF5Manager(filePath, hdf5.H5F_ACC_RDONLY);
         channels.clear();

         ArrayList<String> topicNames = hdf5Tools.findTopicNames(hdf5Manager.getFile());
         for (String topic : topicNames)
         {
            LogTools.info("[Count: {}]\t Channel Found: {}", hdf5Manager.getCount(topic), topic);
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
      hdf5Tools.loadPointCloud(hdf5Manager.getGroup(namespace), index, points, rows, cols);
   }

   public void loadPoint3DList(String namespace, ArrayList<Point3D> points)
   {
//      executorService.submit(() ->
//      {
         int count = (int) hdf5Manager.getCount(namespace);
         for (int index = 0; index < count; index++)
         {
            float[] pointFloatArray = new float[3 * HDF5Manager.MAX_BUFFER_SIZE];
            loadFloatArray(namespace, index, pointFloatArray);

            for (int i = 0; i < pointFloatArray.length / 3; i++)
            {
               points.add(new Point3D(pointFloatArray[i * 3], pointFloatArray[i * 3 + 1], pointFloatArray[i * 3 + 2]));
            }
         }

         LogTools.info("[{}] Total Point3Ds Loaded: {}", namespace, points.size());
//      });
   }

   public void loadQuaternionList(String namespace, ArrayList<Quaternion> quaternions)
   {
//      executorService.submit(() ->
//        {
           int count = (int) hdf5Manager.getCount(namespace);
           for (int index = 0; index < count; index++)
           {
              float[] pointFloatArray = new float[4 * HDF5Manager.MAX_BUFFER_SIZE];
              loadFloatArray(namespace, index, pointFloatArray);

              for (int i = 0; i < pointFloatArray.length / 4; i++)
              {
                 quaternions.add(new Quaternion(pointFloatArray[i * 4],
                                                pointFloatArray[i * 4 + 1],
                                                pointFloatArray[i * 4 + 2],
                                                pointFloatArray[i * 4 + 3]));
              }
           }

           LogTools.info("[{}] Total Quaternions Loaded: {}", namespace, quaternions.size());
//        });
   }

   public void loadFloatArray(String namespace, int index, float[] array)
   {
      Group group = hdf5Manager.getGroup(namespace);
      hdf5Tools.loadFloatArray(group, index, array);
   }

   public void loadCompressedColor(String namespace, int index, Mat mat)
   {
      Group group = hdf5Manager.getGroup(namespace);
      BytePointer bytePointer = new BytePointer(10);
      hdf5Tools.loadBytes(group, index, bytePointer);
      mat.put(BytedecoOpenCVTools.decompressImageJPGUsingYUV(bytePointer));
   }

   public void loadCompressedDepth(String namespace, int index, BytePointer bytePointer, Mat mat)
   {
      Group group = hdf5Manager.getGroup(namespace);
      hdf5Tools.loadBytes(group, index, bytePointer);

      BytedecoOpenCVTools.decompressDepthPNG(bytePointer, mat);
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
      String defaultLogDirectory = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.toString();
      String logDirectory = System.getProperty("perception.log.directory", defaultLogDirectory);
      String logFileName = "20230216_185255_PerceptionLog.hdf5";

      PerceptionDataLoader loader = new PerceptionDataLoader();
      loader.openLogFile(Paths.get(logDirectory, logFileName).toString());

      long total = loader.getHDF5Manager().getCount(PerceptionLoggerConstants.L515_DEPTH_NAME);


      //ArrayList<Point3D> l515PositionList = new ArrayList<>();
      //loader.loadPoint3DList(PerceptionLoggerConstants.L515_SENSOR_POSITION, l515PositionList);
      //
      //ArrayList<Quaternion> l515OrientationList = new ArrayList<>();
      //loader.loadQuaternionList(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, l515OrientationList);

      //ArrayList<Point3D> ousterPositionList = new ArrayList<>();
      //loader.loadPoint3DList(PerceptionLoggerConstants.OUSTER_SENSOR_POSITION, ousterPositionList);
      //
      //ArrayList<Quaternion> ousterOrientationList = new ArrayList<>();
      //loader.loadQuaternionList(PerceptionLoggerConstants.OUSTER_SENSOR_ORIENTATION, ousterOrientationList);

//      ArrayList<Point3D> mocapPositionList = new ArrayList<>();
//      loader.loadPoint3DList(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION, mocapPositionList);
//
//      ArrayList<Quaternion> mocapOrientationList = new ArrayList<>();
//      loader.loadQuaternionList(PerceptionLoggerConstants.MOCAP_RIGID_BODY_ORIENTATION, mocapOrientationList);
//
//      for (int i = 0; i < mocapPositionList.size(); i++)
//      {
//         if ((i % HDF5Manager.MAX_BUFFER_SIZE) == HDF5Manager.MAX_BUFFER_SIZE - 1)
//         {
//            if (i == mocapPositionList.size() - 1)
//            {
//               mocapPositionList.get(i).set(mocapPositionList.get(i - 1));
//            }
//            else
//            {
//               Point3D point1 = mocapPositionList.get(i - 1);
//               Point3D point2 = mocapPositionList.get(i + 1);
//               mocapPositionList.get(i)
//                                .set(EuclidCoreTools.interpolate(point1.getX(), point2.getX(), 0.5f),
//                                     EuclidCoreTools.interpolate(point1.getY(), point2.getY(), 0.5f),
//                                     EuclidCoreTools.interpolate(point1.getZ(), point2.getZ(), 0.5f));
//            }
//         }
//         LogTools.info("[{}] Point3D: {}", i, mocapPositionList.get(i));
//      }
//
//      for (int i = 0; i < mocapOrientationList.size(); i++)
//      {
//         if ((i % HDF5Manager.MAX_BUFFER_SIZE) == HDF5Manager.MAX_BUFFER_SIZE - 1)
//         {
//            if (i == mocapOrientationList.size() - 1)
//            {
//               mocapOrientationList.get(i).set(mocapOrientationList.get(i - 1));
//            }
//            else
//            {
//               Quaternion point1 = mocapOrientationList.get(i - 1);
//               Quaternion point2 = mocapOrientationList.get(i + 1);
//               mocapOrientationList.get(i)
//                                   .set(EuclidCoreTools.interpolate(point1.getX(), point2.getX(), 0.5f),
//                                        EuclidCoreTools.interpolate(point1.getY(), point2.getY(), 0.5f),
//                                        EuclidCoreTools.interpolate(point1.getZ(), point2.getZ(), 0.5f),
//                                        EuclidCoreTools.interpolate(point1.getS(), point2.getS(), 0.5f));
//            }
//         }
//         LogTools.info("[{}] Quaternion: {}", i, mocapOrientationList.get(i));
//      }

         BytePointer bytePointer = new BytePointer(1000000);

         Mat colorImage = new Mat();
         Mat depthImage = new Mat(768, 1024, opencv_core.CV_16UC1);
            LogTools.info("Total Images: {}", total);

            for (int i = 0; i < total; i++)
            {
               //         points.clear();
               //         loader.loadPoint3DList(PerceptionLoggerConstants.L515_SENSOR_POSITION, i, points);

               LogTools.info("Loading Index: {}/{}", i, 10);

               loader.loadCompressedDepth(PerceptionLoggerConstants.L515_DEPTH_NAME, i, bytePointer, depthImage);

               long begin_load = System.nanoTime();
//               loader.loadCompressedImage("/image/", i, colorImage);
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

               BytedecoOpenCVTools.displayDepth("L515 Depth", depthImage, 1);

               int code = opencv_highgui.waitKeyEx(30);
               if (code == 113)
               {
                  System.exit(0);
               }
            }
   }
}
