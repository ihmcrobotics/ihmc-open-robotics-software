package us.ihmc.perception.logging;

import net.jpountz.lz4.LZ4Factory;
import net.jpountz.lz4.LZ4FastDecompressor;
import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;

import java.nio.FloatBuffer;
import java.util.Arrays;

import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKey;

public class PerceptionDataLoader
{
   private HDF5Manager hdf5Manager;

   private final LZ4FastDecompressor lz4Decompressor = LZ4Factory.nativeInstance().fastDecompressor();

   private int latestSegmentIndex = -1;

   private String filePath;

   public PerceptionDataLoader(String filePath)
   {
      this.filePath = filePath;
      hdf5Manager = new HDF5Manager(filePath, hdf5.H5F_ACC_RDONLY);

   }

   public void loadPointCloud(String namespace, int index, RecyclingArrayList<Point3D32> points)
   {
      HDF5Tools.loadPointCloud(hdf5Manager.getGroup(namespace), index, points);
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
      //ThreadTools.startAThread(()->{
         Mat display = new Mat();

         LogTools.info("Loading Image: {} {}", namespace, index);

         Group group = hdf5Manager.getGroup(namespace);
         byte[] compressedByteArray = HDF5Tools.loadByteArray(group, index);

         Mat decompressedImage = decompressImage(compressedByteArray);

         LogTools.info("Completed Loading Image: {} {} {}", index, compressedByteArray.length);

         imshow("Display", decompressedImage);
         waitKey(30);
      //}, "perception_data_loader -> " + namespace);
   }

   private Mat decompressImage(byte[] dataArray)
   {
      LogTools.info("Decompressing Image: {}", dataArray.length);

      BytePointer messageEncodedBytePointer = new BytePointer(dataArray.length);
      messageEncodedBytePointer.put(dataArray, 0, dataArray.length);
      messageEncodedBytePointer.limit(dataArray.length);

      int colorWidth = 848;
      int colorHeight = 480;

      Mat inputJPEGMat = new Mat(1, 1, opencv_core.CV_8UC1);
      Mat inputYUVI420Mat = new Mat(1, 1, opencv_core.CV_8UC1);

      inputJPEGMat.cols(dataArray.length);
      inputJPEGMat.data(messageEncodedBytePointer);

      // imdecode takes the longest by far out of all this stuff
      opencv_imgcodecs.imdecode(inputJPEGMat, opencv_imgcodecs.IMREAD_UNCHANGED, inputYUVI420Mat);

      Mat outputMat = new Mat();

      synchronized (this)
      {
         // YUV I420 has 1.5 times the height of the image
         colorWidth = inputYUVI420Mat.cols();
         colorHeight = (int) (inputYUVI420Mat.rows() / 1.5f);

         outputMat.rows(colorHeight);
         outputMat.cols(colorWidth);

         //opencv_imgproc.cvtColor(inputYUVI420Mat, outputMat, opencv_imgproc.COLOR_YUV2BGR);
      }
      return inputYUVI420Mat;
   }

   public String getFilePath() {
      return filePath;
   }

   public HDF5Manager getHDF5Manager() {
      return hdf5Manager;
   }

   public static void main(String[] args)
   {
//      BytedecoTools.loadOpenCV();

      String LOG_FILE = System.getProperty("perception.log.file", "/home/quantum/Workspace/Data/Sensor_Logs/experimental.hdf5");
      PerceptionDataLoader loader = new PerceptionDataLoader(LOG_FILE);

      for (int i = 1; i < 400; i++)
      {
         loader.loadImage("/d435/depth/", i, null);
      }
   }
}
