package us.ihmc.perception.logging;

import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoOpenCVTools;

import java.io.File;
import java.nio.FloatBuffer;
import java.util.Arrays;

import static org.bytedeco.opencv.global.opencv_highgui.*;

public class PerceptionDataLoader
{
   private HDF5Manager hdf5Manager;
   private String filePath;

   public PerceptionDataLoader(String filePath)
   {
      this.filePath = filePath;
      hdf5Manager = new HDF5Manager(filePath, hdf5.H5F_ACC_RDONLY);
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

      mat.put(decompressImage(compressedByteArray));

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

   private Mat decompressImage(byte[] dataArray)
   {
      LogTools.info("Decompressing Image: {}", dataArray.length);

      BytePointer messageEncodedBytePointer = new BytePointer(dataArray.length);
      messageEncodedBytePointer.put(dataArray, 0, dataArray.length);
      messageEncodedBytePointer.limit(dataArray.length);

      Mat inputJPEGMat = new Mat(1, 1, opencv_core.CV_8UC1);
      Mat inputYUVI420Mat = new Mat(1, 1, opencv_core.CV_8UC1);

      inputJPEGMat.cols(dataArray.length);
      inputJPEGMat.data(messageEncodedBytePointer);

      // imdecode takes the longest by far out of all this stuff
      opencv_imgcodecs.imdecode(inputJPEGMat, opencv_imgcodecs.IMREAD_UNCHANGED, inputYUVI420Mat);

      Mat outputMat = new Mat((int) (inputYUVI420Mat.rows() / 1.5f), inputYUVI420Mat.cols(), opencv_core.CV_8UC4);
      opencv_imgproc.cvtColor(inputYUVI420Mat, outputMat, opencv_imgproc.COLOR_YUV2RGBA_I420);
      opencv_imgproc.cvtColor(outputMat, outputMat, opencv_imgproc.COLOR_RGBA2RGB);

      return outputMat;
   }

   public String getFilePath()
   {
      return filePath;
   }

   public HDF5Manager getHDF5Manager()
   {
      return hdf5Manager;
   }

   public static void main(String[] args)
   {
      String defaultLogDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;
      String LOG_DIRECTORY = System.getProperty("perception.log.directory", defaultLogDirectory);
      String logFileName = "experimental.hdf5";

      PerceptionDataLoader loader = new PerceptionDataLoader(LOG_DIRECTORY + logFileName);

      long totalColor = loader.getHDF5Manager().getCount("/l515/color/");

      Mat colorImage = new Mat();
      Mat depthImage = new Mat(720, 1280, opencv_core.CV_16UC1);
      LogTools.info("Total Images: {}", totalColor);

      for (int i = 0; i < totalColor; i++)
      {
         LogTools.info("Loading Index: {}/{}", i, totalColor);
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
         LogTools.info("Decompression Time: {} ms",(end_decompress - begin_decompress) / 1e6f);

         imshow("/l515/color", colorImage);
         imshow("/l515/depth", finalDisplayDepth);
         int code = waitKeyEx(300);
         if (code == 113)
         {
            System.exit(0);
         }
      }
   }
}
