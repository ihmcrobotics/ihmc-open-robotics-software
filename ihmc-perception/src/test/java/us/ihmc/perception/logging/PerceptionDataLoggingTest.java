package us.ihmc.perception.logging;

import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.indexer.FloatBufferIndexer;
import org.bytedeco.javacpp.indexer.ShortBufferIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;
import us.ihmc.tools.io.resources.ResourceTools;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.nio.file.Paths;
import java.util.Arrays;

import static us.ihmc.robotics.Assert.assertEquals;

public class PerceptionDataLoggingTest
{
   private HDF5Manager hdf5ManagerReader;
   private HDF5Manager hdf5ManagerWriter;

   @Test
   public void testByteToIntArray()
   {
      byte[] dataArray = {(byte) 0, (byte) 255, (byte) 1, (byte) 3, (byte) 4, (byte) 42, (byte) 153, (byte) 10, (byte) 11, (byte) 13, (byte) 15};

      byte[] dataArrayExtended = Arrays.copyOf(dataArray, dataArray.length + (Integer.BYTES - (dataArray.length % Integer.BYTES)));

      ByteBuffer buffer = ByteBuffer.wrap(dataArrayExtended, 0, dataArrayExtended.length);
      IntBuffer intBuffer = buffer.asIntBuffer();

      int intCount = (int)(dataArrayExtended.length / Integer.BYTES) + 1;

      int[] array = new int[intCount];
      intBuffer.get(array, 0, intCount-1);

      LogTools.info("Input Array: {}", Arrays.toString(dataArrayExtended));
      LogTools.info("Input Array: {}", Arrays.toString(array));
   }

   @Test
   @Disabled
   public void testLoggingByteArray()
   {
      hdf5ManagerWriter = new HDF5Manager("hdf5_test.hdf5", hdf5.H5F_ACC_TRUNC());
      Group writeGroup = hdf5ManagerWriter.getGroup("/test/bytes/");

      byte[] dataArray = {(byte) 0, (byte) 255, (byte) 1, (byte) 3, (byte) 4, (byte) 42, (byte) 153, (byte) 10, (byte) 11, (byte) 13, (byte) 15};
      byte[] dataArrayExtended = Arrays.copyOf(dataArray, dataArray.length + (Integer.BYTES - (dataArray.length % Integer.BYTES)));

      HDF5Tools.storeByteArray(writeGroup, 0, dataArrayExtended, dataArrayExtended.length);

      writeGroup.close();
      hdf5ManagerWriter.getFile().close();

      hdf5ManagerReader = new HDF5Manager("hdf5_test.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.openGroup("/test/bytes/");

      byte[] outputArray = HDF5Tools.loadByteArray(readGroup, 0);

      LogTools.info("Input Array: {}", Arrays.toString(dataArrayExtended));
      LogTools.info("Output Array: {}", Arrays.toString(outputArray));

      for (int i = 0; i < dataArrayExtended.length; i++)
      {
         assertEquals(dataArrayExtended[i], outputArray[i]);
      }
      hdf5ManagerReader.getFile().close();
   }

   @Test
   @Disabled
   public void testLoggingLargeByteArray()
   {
      hdf5ManagerWriter = new HDF5Manager("hdf5_test.hdf5", hdf5.H5F_ACC_TRUNC());
      Group writeGroup = hdf5ManagerWriter.getGroup("/test/bytes/");

      byte[] dataArray = new byte[40];
      for (int i = 0; i < dataArray.length; i++)
      {
         dataArray[i] = (byte) i;
      }

      long begin = System.currentTimeMillis();

      System.out.println(Arrays.toString(dataArray));

      HDF5Tools.storeByteArray(writeGroup, 0, dataArray, dataArray.length);

      long intermediate = System.currentTimeMillis();

      writeGroup.close();
      hdf5ManagerWriter.getFile().close();

      hdf5ManagerReader = new HDF5Manager("hdf5_test.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.openGroup("/test/bytes/");

      byte[] outputArray = HDF5Tools.loadByteArray(readGroup, 0);

      long end = System.currentTimeMillis();
      LogTools.info("Logging Took: {} ms", intermediate - begin);
      LogTools.info("Loading Took: {} ms", end - intermediate);

      System.out.println(Arrays.toString(outputArray));

      for (int i = 0; i < dataArray.length; i++)
      {
         assertEquals(dataArray[i], outputArray[i]);
      }
      hdf5ManagerReader.getFile().close();
   }

   @Test
   @Disabled
   public void testLoggingIntArray()
   {
      hdf5ManagerWriter = new HDF5Manager("hdf5_test.hdf5", hdf5.H5F_ACC_TRUNC());
      Group writeGroup = hdf5ManagerWriter.getGroup("/test/ints/");

      int[] dataArray = {0, 255, 1, 3, 4, 42, 153};

      HDF5Tools.storeIntArray(writeGroup, 0, dataArray, dataArray.length);

      writeGroup.close();
      hdf5ManagerWriter.getFile().close();

      hdf5ManagerReader = new HDF5Manager("hdf5_test.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.getGroup("/test/ints/");
      int[] outputArray = HDF5Tools.loadIntArray(readGroup, 0);

      assertEquals(dataArray.length, 7);
      assertEquals(outputArray.length, 7);

      for (int i = 0; i < dataArray.length; i++)
      {
         assertEquals(dataArray[i], outputArray[i]);
      }
      hdf5ManagerReader.getFile().close();
   }

   @Test
   @Disabled
   public void testCompressedFloatDepthLoggingPNG()
   {
      WorkspaceDirectory resourcesDirectory = new WorkspaceDirectory("ihmc-open-robotics-software", "ihmc-perception/src/slam-wrapper/resources");
      hdf5ManagerWriter = new HDF5Manager(resourcesDirectory.getPathNecessaryForClasspathLoading() + "hdf5_test.hdf5", hdf5.H5F_ACC_TRUNC());

      Mat depthFloat = new Mat(128, 128, opencv_core.CV_32FC1);
      depthFloat.put(new Scalar(1.234));

      BytePointer compressedDepthPointer = new BytePointer();
      BytedecoOpenCVTools.compressImagePNG(depthFloat, compressedDepthPointer);

      byte[] dataArray = new byte[compressedDepthPointer.asBuffer().remaining() + 4];
      compressedDepthPointer.asBuffer().get(dataArray, 0, dataArray.length - 4);

      LogTools.info("Raw Size: {}, Compressed Size: {}", depthFloat.rows() * depthFloat.cols() * 4, dataArray.length);

      Group writeGroup = hdf5ManagerWriter.getGroup("/test/bytes/");

//      LogTools.info("PNG Stored: [{}] -> {}", dataArray.length, Arrays.toString(dataArray));

      HDF5Tools.storeByteArray(writeGroup, 0, dataArray, dataArray.length);

      writeGroup.close();
      hdf5ManagerWriter.getFile().close();

      hdf5ManagerReader = new HDF5Manager("hdf5_test.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.openGroup("/test/bytes/");

      byte[] pngCompressedBytes = HDF5Tools.loadByteArray(readGroup, 0);

//      LogTools.info("PNG Loaded: [{}] -> {}", pngCompressedBytes.length, Arrays.toString(pngCompressedBytes));


      Mat finalDepthUC4 = new Mat(128, 128, opencv_core.CV_8UC4);
      BytedecoOpenCVTools.decompressDepthPNG(pngCompressedBytes, finalDepthUC4);

      Mat finalDepthFloat = new Mat(128, 128, opencv_core.CV_32FC1, finalDepthUC4.data());


      FloatBufferIndexer indexer = new FloatBufferIndexer(finalDepthFloat.getFloatBuffer());
      FloatBufferIndexer indexerActual = new FloatBufferIndexer(depthFloat.getFloatBuffer());

      float diff = 0.0f;
      for(int i = 0; i<128; i++)
      {
         for(int j = 0; j<128; j++)
         {
            diff += Math.abs(indexer.get(i*128 + j) - indexerActual.get(i*128 + j));

            //LogTools.info("Depth ({} {}): {}", i, j, indexer.get(i*128 + j));
            //LogTools.info("Actual ({} {}): {}", i, j, indexerActual.get(i*128 + j));

         }
      }

      assertEquals(0.0, diff, 1e-5);
   }

   @Test
   @Disabled
   public void testCompressedDepthMapLoggingPNG()
   {

      hdf5ManagerWriter = new HDF5Manager("hdf5_test.hdf5", hdf5.H5F_ACC_TRUNC());

      Mat depth = new Mat(128, 128, opencv_core.CV_16UC1);
      depth.put(new Scalar(12345));

      BytePointer compressedDepthPointer = new BytePointer();
      BytedecoOpenCVTools.compressImagePNG(depth, compressedDepthPointer);

      byte[] dataArray = new byte[compressedDepthPointer.asBuffer().remaining()];
      compressedDepthPointer.asBuffer().get(dataArray, 0, dataArray.length);

//      LogTools.info("Raw Size: {}, Compressed Size: {}", depth.rows() * depth.cols() * 4, dataArray.length);

      Group writeGroup = hdf5ManagerWriter.getGroup("/test/bytes/");

//      LogTools.info("PNG Stored: [{}] -> {}", dataArray.length, Arrays.toString(dataArray));

      HDF5Tools.storeByteArray(writeGroup, 0, dataArray, dataArray.length);

      writeGroup.close();
      hdf5ManagerWriter.getFile().close();

      hdf5ManagerReader = new HDF5Manager("hdf5_test.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.openGroup("/test/bytes/");

      byte[] pngCompressedBytes = HDF5Tools.loadByteArray(readGroup, 0);

//      LogTools.info("PNG Loaded: [{}] -> {}", pngCompressedBytes.length, Arrays.toString(pngCompressedBytes));


      Mat finalDepth16UC1 = new Mat(128, 128, opencv_core.CV_16UC1);
      BytedecoOpenCVTools.decompressDepthPNG(pngCompressedBytes, finalDepth16UC1);

      ShortBufferIndexer indexer = new ShortBufferIndexer(finalDepth16UC1.getShortBuffer());
      ShortBufferIndexer indexerActual = new ShortBufferIndexer(depth.getShortBuffer());

      short diff = 0;
      for(int i = 0; i<128; i++)
      {
         for(int j = 0; j<128; j++)
         {
            diff += Math.abs(indexer.get(i*128 + j) - indexerActual.get(i*128 + j));

            //LogTools.info("Depth ({} {}): {}", i, j, indexer.get(i*128 + j));
            //LogTools.info("Actual ({} {}): {}", i, j, indexerActual.get(i*128 + j));

         }
      }

      assertEquals(0.0, diff, 1e-5);
   }

   @Test
   @Disabled
   public void testCompressedDepthLoggingJPG()
   {

//      hdf5ManagerWriter = new HDF5Manager("hdf5_test.hdf5", hdf5.H5F_ACC_TRUNC());

      Mat depth = new Mat(128, 128, opencv_core.CV_16UC1);
      depth.put(new Scalar(12345));

      BytePointer compressedDepthPointer = new BytePointer();
      BytedecoOpenCVTools.compressDepthJPG(depth, compressedDepthPointer);

      byte[] dataArray = new byte[compressedDepthPointer.asBuffer().remaining()];
      compressedDepthPointer.asBuffer().get(dataArray, 0, dataArray.length);

//      LogTools.info("Raw Size: {}, Compressed Size: {}", depth.rows() * depth.cols() * 4, dataArray.length);
//
//      Group writeGroup = hdf5ManagerWriter.getGroup("/test/bytes/");
//
//      LogTools.info("PNG Stored: [{}] -> {}", dataArray.length, Arrays.toString(dataArray));
//
//      HDF5Tools.storeByteArray(writeGroup, 0, dataArray, dataArray.length);
//
//      writeGroup.close();
//      hdf5ManagerWriter.getFile().close();
//
//      hdf5ManagerReader = new HDF5Manager("hdf5_test.hdf5", hdf5.H5F_ACC_RDONLY());
//      Group readGroup = hdf5ManagerReader.openGroup("/test/bytes/");
//
//      byte[] pngCompressedBytes = HDF5Tools.loadByteArray(readGroup, 0);

//      LogTools.info("PNG Loaded: [{}] -> {}", dataArray.length, Arrays.toString(dataArray));


      Mat finalDepth16UC1 = new Mat(128, 128, opencv_core.CV_16UC1);
      BytedecoOpenCVTools.decompressJPG(dataArray, finalDepth16UC1);

      ShortBufferIndexer indexer = new ShortBufferIndexer(finalDepth16UC1.getShortBuffer());
      ShortBufferIndexer indexerActual = new ShortBufferIndexer(depth.getShortBuffer());

      long diff = 0;
      for(int i = 0; i<128; i++)
      {
         for(int j = 0; j<128; j++)
         {
            diff += Math.abs(indexer.get(i*128 + j) - indexerActual.get(i*128 + j));

//            LogTools.info("Depth ({} {}): {}", i, j, indexer.get(i*128 + j));
//            LogTools.info("Actual ({} {}): {}", i, j, indexerActual.get(i*128 + j));

         }
      }

      assertEquals(0, diff, 1e-5);
   }

   @Test
   @Disabled
   public void testDepthCompressionJPG()
   {

      Mat depth = new Mat(128, 128, opencv_core.CV_16UC1);
      depth.put(new Scalar(12345));

      BytePointer compressedDepthPointer = new BytePointer();
      BytedecoOpenCVTools.compressDepthJPG(depth, compressedDepthPointer);

      byte[] dataArray = new byte[compressedDepthPointer.asBuffer().remaining()];
      compressedDepthPointer.asBuffer().get(dataArray, 0, dataArray.length);

//      LogTools.info("PNG Loaded: [{}] -> {}", dataArray.length, Arrays.toString(dataArray));

      Mat finalDepth16UC1 = new Mat(128, 128, opencv_core.CV_16UC1);
      BytedecoOpenCVTools.decompressJPG(dataArray, finalDepth16UC1);

      ShortBufferIndexer indexer = new ShortBufferIndexer(finalDepth16UC1.getShortBuffer());
      ShortBufferIndexer indexerActual = new ShortBufferIndexer(depth.getShortBuffer());

      long diff = 0;
      for(int i = 0; i<128; i++)
      {
         for(int j = 0; j<128; j++)
         {
            diff += Math.abs(indexer.get(i*128 + j) - indexerActual.get(i*128 + j));

//            LogTools.info("Depth ({} {}): {}", i, j, indexer.get(i*128 + j));
//            LogTools.info("Actual ({} {}): {}", i, j, indexerActual.get(i*128 + j));

         }
      }

      assertEquals(0, diff, 1e-5);
   }

   @Test
   @Disabled
   public void testCompressedDepthMapLoggingPNG()
   {

      hdf5ManagerWriter = new HDF5Manager("hdf5_test.hdf5", hdf5.H5F_ACC_TRUNC());

      Mat depth = new Mat(128, 128, opencv_core.CV_16UC1);
      depth.put(new Scalar(12345));

      BytePointer compressedDepthPointer = new BytePointer();
      BytedecoOpenCVTools.compressImagePNG(depth, compressedDepthPointer);

      byte[] dataArray = new byte[compressedDepthPointer.asBuffer().remaining()];
      compressedDepthPointer.asBuffer().get(dataArray, 0, dataArray.length);

      //      LogTools.info("Raw Size: {}, Compressed Size: {}", depth.rows() * depth.cols() * 4, dataArray.length);

      Group writeGroup = hdf5ManagerWriter.getGroup("/test/bytes/");

      //      LogTools.info("PNG Stored: [{}] -> {}", dataArray.length, Arrays.toString(dataArray));

      HDF5Tools.storeByteArray(writeGroup, 0, dataArray, dataArray.length);

      writeGroup.close();
      hdf5ManagerWriter.getFile().close();

      hdf5ManagerReader = new HDF5Manager("hdf5_test.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.openGroup("/test/bytes/");

      byte[] pngCompressedBytes = HDF5Tools.loadByteArray(readGroup, 0);

      //      LogTools.info("PNG Loaded: [{}] -> {}", pngCompressedBytes.length, Arrays.toString(pngCompressedBytes));


      Mat finalDepth16UC1 = new Mat(128, 128, opencv_core.CV_16UC1);
      BytedecoOpenCVTools.decompressDepthPNG(pngCompressedBytes, finalDepth16UC1);

      ShortBufferIndexer indexer = new ShortBufferIndexer(finalDepth16UC1.getShortBuffer());
      ShortBufferIndexer indexerActual = new ShortBufferIndexer(depth.getShortBuffer());

      short diff = 0;
      for(int i = 0; i<128; i++)
      {
         for(int j = 0; j<128; j++)
         {
            diff += Math.abs(indexer.get(i*128 + j) - indexerActual.get(i*128 + j));

            //LogTools.info("Depth ({} {}): {}", i, j, indexer.get(i*128 + j));
            //LogTools.info("Actual ({} {}): {}", i, j, indexerActual.get(i*128 + j));

         }
      }

      assertEquals(0.0, diff, 1e-5);
   }

}
