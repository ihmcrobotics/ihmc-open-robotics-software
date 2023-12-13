package us.ihmc.perception.logging;

import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.LongPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
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
      }
      else
      {
         LogTools.warn("Log file does not exist: {}", filePath);
      }
   }

   public void addByteChannel(String channelName, int frameSize, int blockSize)
   {
      channels.put(channelName, new PerceptionLogChannel(channelName, 0, 0, frameSize, blockSize, new BytePointer(PerceptionLoggerConstants.BYTE_BUFFER_SIZE)));
   }

   public void addFloatChannel(String channelName, int frameSize, int blockSize)
   {
      channels.put(channelName,
                   new PerceptionLogChannel(channelName, 0, 0, frameSize, blockSize, new FloatPointer(PerceptionLoggerConstants.FLOAT_BUFFER_SIZE)));
   }

   public void addLongChannel(String channelName, int frameSize, int blockSize)
   {
      channels.put(channelName, new PerceptionLogChannel(channelName, 0, 0, frameSize, blockSize, new LongPointer(PerceptionLoggerConstants.LONG_BUFFER_SIZE)));
   }

   public void addIntChannel(String channelName, int frameSize, int blockSize)
   {
      channels.put(channelName, new PerceptionLogChannel(channelName, 0, 0, frameSize, blockSize, new IntPointer(PerceptionLoggerConstants.INT_BUFFER_SIZE)));
   }

   public void addImageChannel(String channelName)
   {
      channels.put(channelName,
                   new PerceptionLogChannel(channelName,
                                            0,
                                            0,
                                            PerceptionLoggerConstants.COMPRESSED_IMAGE_BUFFER_SIZE,
                                            1,
                                            new BytePointer(PerceptionLoggerConstants.COMPRESSED_IMAGE_BUFFER_SIZE)));
   }

   public void loadPoint3DList(String namespace, ArrayList<Point3D> points)
   {
      loadPoint3DList(namespace, points, PerceptionLoggerConstants.DEFAULT_BLOCK_SIZE);
   }

   public void loadPoint3DList(String namespace, ArrayList<Point3D> points, int blockSize)
   {
      //      executorService.submit(() ->
      //      {
      int count = (int) hdf5Manager.getCount(namespace);
      LogTools.info("[{}] Total Point3D Blocks: {}", namespace, count);

      for (int index = 0; index < count; index++)
      {
         float[] pointFloatArray = new float[3 * blockSize];
         loadFloatArray(namespace, index, pointFloatArray);

         for (int i = 0; i < pointFloatArray.length / 3; i++)
         {
            //LogTools.info("Point: {}, {}, {}", pointFloatArray[i * 3], pointFloatArray[i * 3 + 1], pointFloatArray[i * 3 + 2]);
            points.add(new Point3D(pointFloatArray[i * 3], pointFloatArray[i * 3 + 1], pointFloatArray[i * 3 + 2]));
         }
      }

      LogTools.info("[{}] Total Point3Ds Loaded: {}", namespace, points.size());
      //      });
   }

   public void loadQuaternionList(String namespace, ArrayList<Quaternion> quaternions)
   {
      loadQuaternionList(namespace, quaternions, PerceptionLoggerConstants.DEFAULT_BLOCK_SIZE);
   }

   public void loadQuaternionList(String namespace, ArrayList<Quaternion> quaternions, int blockSize)
   {
      //      executorService.submit(() ->
      //        {
      int count = (int) hdf5Manager.getCount(namespace);
      for (int index = 0; index < count; index++)
      {
         float[] pointFloatArray = new float[4 * blockSize];
         loadFloatArray(namespace, index, pointFloatArray);

         for (int i = 0; i < pointFloatArray.length / 4; i++)
         {
            quaternions.add(new Quaternion(pointFloatArray[i * 4], pointFloatArray[i * 4 + 1], pointFloatArray[i * 4 + 2], pointFloatArray[i * 4 + 3]));
         }
      }

      LogTools.info("[{}] Total Quaternions Loaded: {}", namespace, quaternions.size());
      //        });
   }

   public void loadFloatArray(String namespace, int index, float[] array)
   {
      Group group = hdf5Manager.openOrGetGroup(namespace);
      hdf5Tools.loadFloatArray(group, index, array);
   }

   public void loadCompressedColor(String namespace, int index, Mat mat)
   {
      Group group = hdf5Manager.openOrGetGroup(namespace);
      BytePointer bytePointer = new BytePointer(10);
      hdf5Tools.loadBytes(group, index, bytePointer);
      mat.put(OpenCVTools.decompressImageJPGUsingYUV(bytePointer));
   }

   public void loadCompressedDepth(String namespace, int index, BytePointer bytePointer, Mat mat)
   {
      Group group = hdf5Manager.openOrGetGroup(namespace);
      hdf5Tools.loadBytes(group, index, bytePointer);

      OpenCVTools.decompressDepthPNG(bytePointer, mat);
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

   public void closeLogFile()
   {
      hdf5Manager.closeFile();
   }

   public static void main(String[] args)
   {
      String defaultLogDirectory = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.toString();
      String logDirectory = System.getProperty("perception.log.directory", defaultLogDirectory);
      String logFileName = "20230228_145121_PerceptionLog.hdf5";

      PerceptionDataLoader loader = new PerceptionDataLoader();
      loader.openLogFile(Paths.get(logDirectory, logFileName).toString());

      long total = loader.getHDF5Manager().getCount(PerceptionLoggerConstants.L515_DEPTH_NAME);

      BytePointer bytePointer = new BytePointer(PerceptionLoggerConstants.FLOAT_BUFFER_SIZE);

      Mat depthImage = new Mat(768, 1280, opencv_core.CV_16UC1);
      LogTools.info("Total Images: {}", total);

      for (int i = 0; i < total; i++)
      {
         LogTools.info("Loading Index: {}/{}", i, 10);
         loader.loadCompressedDepth(PerceptionLoggerConstants.L515_DEPTH_NAME, i, bytePointer, depthImage);
         PerceptionDebugTools.displayDepth("L515 Depth", depthImage, 1);
      }
   }
}
