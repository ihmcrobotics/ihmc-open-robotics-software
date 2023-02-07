package us.ihmc.perception.logging;

import gnu.trove.list.array.TFloatArrayList;
import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.indexer.FloatBufferIndexer;
import org.bytedeco.javacpp.indexer.ShortBufferIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoOpenCVTools;

import java.io.File;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
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
   public void testLoggingByteArray() throws InterruptedException
   {
      hdf5ManagerWriter = new HDF5Manager("hdf5_test_byte_array.hdf5", hdf5.H5F_ACC_TRUNC());
      Group writeGroup = hdf5ManagerWriter.getGroup("/test/bytes/");

      byte[] dataArray = {(byte) 0, (byte) 255, (byte) 1, (byte) 3, (byte) 4, (byte) 42, (byte) 153, (byte) 10, (byte) 11, (byte) 13, (byte) 15};
      byte[] dataArrayExtended = Arrays.copyOf(dataArray, dataArray.length + (Integer.BYTES - (dataArray.length % Integer.BYTES)));

      HDF5Tools.storeByteArray(writeGroup, 0, dataArrayExtended, dataArrayExtended.length);

      writeGroup.close();
      hdf5ManagerWriter.getFile().close();

      hdf5ManagerReader = new HDF5Manager("hdf5_test_byte_array.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.openGroup("/test/bytes/");

      byte[] outputArray = HDF5Tools.loadByteArray(readGroup, 0);

      LogTools.info("Input Array: {}", Arrays.toString(dataArrayExtended));
      LogTools.info("Output Array: {}", Arrays.toString(outputArray));

      for (int i = 0; i < dataArrayExtended.length; i++)
      {
         assertEquals(dataArrayExtended[i], outputArray[i]);
      }
      hdf5ManagerWriter.closeFile();
      hdf5ManagerReader.closeFile();
   }

   @Test
   public void testLoggingLargeByteArray() throws InterruptedException
   {
      hdf5ManagerWriter = new HDF5Manager("hdf5_test_large_bytes.hdf5", hdf5.H5F_ACC_TRUNC());
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

      hdf5ManagerReader = new HDF5Manager("hdf5_test_large_bytes.hdf5", hdf5.H5F_ACC_RDONLY());
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
      hdf5ManagerWriter.closeFile();
      hdf5ManagerReader.closeFile();
   }

   @Test
   public void testLoggingIntArray() throws InterruptedException
   {
      hdf5ManagerWriter = new HDF5Manager("hdf5_test_int_array.hdf5", hdf5.H5F_ACC_TRUNC());
      Group writeGroup = hdf5ManagerWriter.getGroup("/test/ints/");

      int[] dataArray = {0, 255, 1, 3, 4, 42, 153};

      HDF5Tools.storeIntArray(writeGroup, 0, dataArray, dataArray.length);

      writeGroup.close();
      hdf5ManagerWriter.getFile().close();

      hdf5ManagerReader = new HDF5Manager("hdf5_test_int_array.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.getGroup("/test/ints/");
      int[] outputArray = HDF5Tools.loadIntArray(readGroup, 0);

      assertEquals(dataArray.length, 7);
      assertEquals(outputArray.length, 7);

      for (int i = 0; i < dataArray.length; i++)
      {
         assertEquals(dataArray[i], outputArray[i]);
      }
      hdf5ManagerWriter.closeFile();
      hdf5ManagerReader.closeFile();
   }

   @Test
   public void testCompressedFloatDepthLoggingPNG() throws InterruptedException
   {
      hdf5ManagerWriter = new HDF5Manager("hdf5_test_depth_png.hdf5", hdf5.H5F_ACC_TRUNC());

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

      hdf5ManagerReader = new HDF5Manager("hdf5_test_depth_png.hdf5", hdf5.H5F_ACC_RDONLY());
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

      hdf5ManagerWriter.closeFile();
      hdf5ManagerReader.closeFile();

      assertEquals(0.0, diff, 1e-5);
   }

   @Test
   public void testCompressedDepthMapLoggingPNG() throws InterruptedException
   {
      hdf5ManagerWriter = new HDF5Manager("hdf5_test_depth_png_2.hdf5", hdf5.H5F_ACC_TRUNC());

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

      hdf5ManagerReader = new HDF5Manager("hdf5_test_depth_png_2.hdf5", hdf5.H5F_ACC_RDONLY());
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

      hdf5ManagerWriter.closeFile();
      hdf5ManagerReader.closeFile();

      assertEquals(0.0, diff, 1e-5);
   }

   @Test
   @Disabled
   public void testCompressedDepthLoggingJPG() throws InterruptedException
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
   public void testDepthCompressionJPG() throws InterruptedException
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
   public void testStoreAndLoadFloatArray() throws InterruptedException
   {
      hdf5ManagerWriter = new HDF5Manager("hdf5_test_float_array.hdf5", hdf5.H5F_ACC_TRUNC());

      Group writeGroup = hdf5ManagerWriter.getGroup("/test/bytes/");

      float[] floatArray = new float[]{12.3f, 32.1f, 43.1f, 32.43f};
      TFloatArrayList dataArray = new TFloatArrayList(floatArray);

      HDF5Tools.storeFloatArray2D(writeGroup, 0, dataArray, 1, dataArray.size());

      writeGroup.close();
      hdf5ManagerWriter.getFile().close();

      hdf5ManagerReader = new HDF5Manager("hdf5_test_float_array.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.openGroup("/test/bytes/");

      float[] loadedFloats = HDF5Tools.loadFloatArray(readGroup, 0);

      float diff = 0;
      for(int i = 0; i<loadedFloats.length; i++)
      {
         LogTools.info("Stored: {}, Loaded: {}", floatArray[i], loadedFloats[i]);
         diff += Math.abs(loadedFloats[i] - floatArray[i]);
      }

      hdf5ManagerWriter.closeFile();
      hdf5ManagerReader.closeFile();

      assertEquals(0.0, diff, 1e-5);
   }

   @Test
   @Disabled
   public void testNativeHDF5API()
   {
      File file = new File("hdf5_test.hdf5");
      long fileId = hdf5.H5Fcreate("hdf5_test.hdf5", hdf5.H5F_ACC_TRUNC, hdf5.H5P_DEFAULT, hdf5.H5P_DEFAULT());

      // Create a link creation property list
      long lcpl_id = hdf5.H5Pcreate(hdf5.H5P_LINK_CREATE);
      hdf5.H5Pset_create_intermediate_group(lcpl_id, 1);

      // Create a dataset creation property list
      long dcpl_id = hdf5.H5Pcreate(hdf5.H5P_DATASET_CREATE);
      hdf5.H5Pset_layout(dcpl_id, hdf5.H5D_CHUNKED);
      hdf5.H5Pset_chunk(dcpl_id, 1, new long[] { 4 });

      // Create a dataset access property list
      long dapl_id = hdf5.H5Pcreate(hdf5.H5P_DATASET_ACCESS);
      hdf5.H5Pset_chunk_cache(dapl_id, 1048576, 100, 1048576);

      // Create a new dataset
      long dsSpaceId = hdf5.H5Screate_simple(1, new long[] { 1 }, null);
      long datasetId = hdf5.H5Dcreate2(fileId, "/test/position", hdf5.H5T_NATIVE_FLOAT(), dsSpaceId, lcpl_id, dcpl_id, dapl_id);

      long spaceId = hdf5.H5Screate_simple(1, new long[] { 1 }, null);
      long attributeId = hdf5.H5Acreate2(datasetId, "attribute_name", hdf5.H5T_STD_I32LE, spaceId, hdf5.H5P_DEFAULT, hdf5.H5P_DEFAULT);
      hdf5.H5Awrite(attributeId, hdf5.H5T_NATIVE_INT, new IntPointer(new int[] {123 }));
      hdf5.H5Aclose(attributeId);
      hdf5.H5Dclose(datasetId);

      long readFileId = hdf5.H5Fopen("hdf5_test.hdf5", hdf5.H5F_ACC_RDONLY, hdf5.H5P_DEFAULT);
      long readDatasetId = hdf5.H5Dopen2(readFileId, "/test/position", hdf5.H5P_DEFAULT);

      long readAttributeId = hdf5.H5Aopen(readDatasetId, "attribute_name", hdf5.H5P_DEFAULT());
      int[] data = new int[1];
      hdf5.H5Aread(readAttributeId, hdf5.H5T_NATIVE_INT, new IntPointer(data));
      System.out.println("Attribute value: " + data[0]);
      hdf5.H5Aclose(readAttributeId);
      hdf5.H5Dclose(readDatasetId);
      hdf5.H5Fclose(readFileId);

      assertEquals(123, data[0], 1e-5);
   }
}
