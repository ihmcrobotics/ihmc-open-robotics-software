package us.ihmc.perception.logging;

import org.bytedeco.hdf5.*;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.indexer.FloatBufferIndexer;
import org.bytedeco.javacpp.indexer.ShortBufferIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.opencv.OpenCVTools;

import java.io.File;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
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
   public void testLoggingByteArrayAsInts()
   {
      HDF5Tools hdf5Tools = new HDF5Tools();
      hdf5ManagerWriter = new HDF5Manager("hdf5_test_byte_array_as_ints.hdf5", hdf5.H5F_ACC_TRUNC());
      Group writeGroup = hdf5ManagerWriter.createOrGetGroup("/test/bytes/");

      byte[] dataArray = {(byte) 0, (byte) 255, (byte) 1, (byte) 3, (byte) 4, (byte) 42, (byte) 153, (byte) 10, (byte) 11, (byte) 13, (byte) 15};
      byte[] dataArrayExtended = Arrays.copyOf(dataArray, dataArray.length + (Integer.BYTES - (dataArray.length % Integer.BYTES)));

      hdf5Tools.storeByteArray(writeGroup, 0, dataArrayExtended, dataArrayExtended.length);

      writeGroup._close();
      hdf5ManagerWriter.getFile()._close();

      hdf5ManagerReader = new HDF5Manager("hdf5_test_byte_array_as_ints.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.openOrGetGroup("/test/bytes/");

      byte[] outputArray = hdf5Tools.loadByteArray(readGroup, 0);

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
   public void testLoggingByteArray() throws InterruptedException
   {
      HDF5Tools hdf5Tools = new HDF5Tools();

      H5File writeFile = new H5File("hdf5_test_byte_array.hdf5", hdf5.H5F_ACC_TRUNC());
      Group writeGroup = writeFile.createGroup("test");

      //byte[] dataArray = {(byte) 123, (byte) 255, (byte) 1, (byte) 3, (byte) 4, (byte) 42, (byte) 153, (byte) 10, (byte) 11, (byte) 13, (byte) 15};
      byte[] dataArray = new byte[100];

      for (int i = 0; i < dataArray.length; i++)
      {
         dataArray[i] = (byte) i;
      }

      BytePointer dataBytePointer = new BytePointer(dataArray);

      long size = dataBytePointer.limit();

      LogTools.info("Store Byte Array: Index: {} Size: {}", 0, size);
      long[] dims = {size};

      DataType dataType = new DataType(PredType.NATIVE_B8());
      DataSpace dataSpace = new DataSpace(1, dims);
      DataSet dataSet = writeGroup.createDataSet(String.valueOf(0), dataType, dataSpace);

      dataSet.write((Pointer) dataBytePointer, dataType);

      //hdf5Tools.storeBytes(writeGroup, 0, dataBytePointer);

      writeFile._close();
      //
      LogTools.info("File closed");
      //
      LogTools.info("Opening file");
      H5File readFile = new H5File("hdf5_test_byte_array.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = readFile.openGroup( "test");

      BytePointer destinationBytePointer = new BytePointer(0);

      DataSet readDataSet = readGroup.openDataSet(String.valueOf(0));

      long readSize = readDataSet.getInMemDataSize();
      if(readSize > destinationBytePointer.capacity())
      {
         LogTools.warn("Byte array is too small to hold the data. Resizing to {} from {}", readSize, destinationBytePointer.capacity());
         //destinationBytePointer.capacity(2 * readSize);
         destinationBytePointer = new BytePointer(2 * readSize);
      }

      DataType readDataType = new DataType(PredType.NATIVE_B8());
      readDataSet.read((Pointer) destinationBytePointer, readDataType);

      //hdf5Tools.loadBytes(readGroup, 0, destinationBytePointer);

      for (int i = 0; i < dataBytePointer.limit(); i++)
      {
         LogTools.info("i: {}, data: {}, destination: {}", i, dataBytePointer.get(i), destinationBytePointer.get(i));
         assertEquals(dataBytePointer.get(i), destinationBytePointer.get(i));
      }

      readFile._close();
   }


   @Test
   public void testCompressedDepthMapLoggingPNGRawBytes() throws InterruptedException
   {
      HDF5Tools hdf5Tools = new HDF5Tools();
      hdf5ManagerWriter = new HDF5Manager("hdf5_test_depth_png_3.hdf5", hdf5.H5F_ACC_TRUNC());

      Mat depth = new Mat(128, 128, opencv_core.CV_16UC1);
      depth.put(new Scalar(12345));

      BytePointer compressedDepthPointer = new BytePointer(0);
      OpenCVTools.compressImagePNG(depth, compressedDepthPointer);

      Group writeGroup = hdf5ManagerWriter.createOrGetGroup("/test/bytes/");

      hdf5Tools.storeBytes(writeGroup, 0, compressedDepthPointer);

      writeGroup._close();
      writeGroup = null;
      hdf5ManagerWriter.closeFile();

      hdf5ManagerReader = new HDF5Manager("hdf5_test_depth_png_3.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.openOrGetGroup("/test/bytes/");

      BytePointer pngCompressedBytes = new BytePointer(compressedDepthPointer.limit());
      hdf5Tools.loadBytes(readGroup, 0, pngCompressedBytes);

      for (int i = 0; i < compressedDepthPointer.limit(); i++)
      {
         LogTools.info("i: {}, data: {}, destination: {}", i, compressedDepthPointer.get(i), pngCompressedBytes.get(i));
         assertEquals(compressedDepthPointer.get(i), pngCompressedBytes.get(i));
      }

      LogTools.info("Loaded Size: {}", pngCompressedBytes.limit());

      Mat finalDepth16UC1 = new Mat(128, 128, opencv_core.CV_16UC1);
      OpenCVTools.decompressDepthPNG(pngCompressedBytes, finalDepth16UC1);

      //readGroup._close();

      hdf5ManagerReader.closeFile();

      ShortBufferIndexer indexer = new ShortBufferIndexer(finalDepth16UC1.getShortBuffer());
      ShortBufferIndexer indexerActual = new ShortBufferIndexer(depth.getShortBuffer());

      short diff = 0;
      for(int i = 0; i<128; i++)
      {
         for(int j = 0; j<128; j++)
         {
            diff += Math.abs(indexer.get(i*128 + j) - indexerActual.get(i*128 + j));

            LogTools.info(String.format("Depth (%d %d): %d %d", i, j, indexer.get(i*128 + j), indexerActual.get(i*128 + j)));
            //LogTools.info("Actual ({} {}): {}", i, j, indexerActual.get(i*128 + j));

         }
      }

      assertEquals(0.0, diff, 1e-5);

      LogTools.info("Finished Test for PNG Raw Bytes");
   }

   @Test
   public void testLoggingLargeByteArrayAsInts()
   {
      HDF5Tools hdf5Tools = new HDF5Tools();
      hdf5ManagerWriter = new HDF5Manager("hdf5_test_large_bytes_as_ints.hdf5", hdf5.H5F_ACC_TRUNC());
      Group writeGroup = hdf5ManagerWriter.createOrGetGroup("/test/bytes/");

      byte[] dataArray = new byte[40];
      for (int i = 0; i < dataArray.length; i++)
      {
         dataArray[i] = (byte) i;
      }

      long begin = System.currentTimeMillis();

      System.out.println(Arrays.toString(dataArray));

      hdf5Tools.storeByteArray(writeGroup, 0, dataArray, dataArray.length);

      long intermediate = System.currentTimeMillis();

      writeGroup._close();
      hdf5ManagerWriter.getFile()._close();

      hdf5ManagerReader = new HDF5Manager("hdf5_test_large_bytes_as_ints.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.openOrGetGroup("/test/bytes/");

      byte[] outputArray = hdf5Tools.loadByteArray(readGroup, 0);

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

   @Disabled
   @Test
   public void testCompressedFloatDepthLoggingPNG()
   {
      HDF5Tools hdf5Tools = new HDF5Tools();
      hdf5ManagerWriter = new HDF5Manager("hdf5_test_depth_png.hdf5", hdf5.H5F_ACC_TRUNC());

      Mat depthFloat = new Mat(128, 128, opencv_core.CV_32FC1);
      depthFloat.put(new Scalar(1.234));

      BytePointer compressedDepthPointer = new BytePointer();
      OpenCVTools.compressImagePNG(depthFloat, compressedDepthPointer);

      byte[] dataArray = new byte[compressedDepthPointer.asBuffer().remaining() + 4];
      compressedDepthPointer.asBuffer().get(dataArray, 0, dataArray.length - 4);

      LogTools.info("Raw Size: {}, Compressed Size: {}", depthFloat.rows() * depthFloat.cols() * 4, dataArray.length);

      Group writeGroup = hdf5ManagerWriter.createOrGetGroup("/test/bytes/");

//      LogTools.info("PNG Stored: [{}] -> {}", dataArray.length, Arrays.toString(dataArray));

      hdf5Tools.storeByteArray(writeGroup, 0, dataArray, dataArray.length);

      writeGroup._close();
      hdf5ManagerWriter.getFile()._close();

      hdf5ManagerReader = new HDF5Manager("hdf5_test_depth_png.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.openOrGetGroup("/test/bytes/");

      byte[] pngCompressedBytes = hdf5Tools.loadByteArray(readGroup, 0);

//      LogTools.info("PNG Loaded: [{}] -> {}", pngCompressedBytes.length, Arrays.toString(pngCompressedBytes));


      Mat finalDepthUC4 = new Mat(128, 128, opencv_core.CV_8UC4);
      OpenCVTools.decompressDepthPNG(new BytePointer(pngCompressedBytes), finalDepthUC4);

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
   public void testCompressedDepthMapLoggingPNG()
   {
      HDF5Tools hdf5Tools = new HDF5Tools();
      hdf5ManagerWriter = new HDF5Manager("hdf5_test_depth_png_2.hdf5", hdf5.H5F_ACC_TRUNC());

      Mat depth = new Mat(128, 128, opencv_core.CV_16UC1);
      depth.put(new Scalar(12345));

      BytePointer compressedDepthPointer = new BytePointer();
      OpenCVTools.compressImagePNG(depth, compressedDepthPointer);

      byte[] dataArray = new byte[compressedDepthPointer.asBuffer().remaining()];
      compressedDepthPointer.asBuffer().get(dataArray, 0, dataArray.length);

//      LogTools.info("Raw Size: {}, Compressed Size: {}", depth.rows() * depth.cols() * 4, dataArray.length);

      Group writeGroup = hdf5ManagerWriter.createOrGetGroup("/test/bytes/");

//      LogTools.info("PNG Stored: [{}] -> {}", dataArray.length, Arrays.toString(dataArray));

      hdf5Tools.storeByteArray(writeGroup, 0, dataArray, dataArray.length);

      writeGroup._close();
      hdf5ManagerWriter.getFile()._close();

      hdf5ManagerReader = new HDF5Manager("hdf5_test_depth_png_2.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.openOrGetGroup("/test/bytes/");

      byte[] pngCompressedBytes = hdf5Tools.loadByteArray(readGroup, 0);

//      LogTools.info("PNG Loaded: [{}] -> {}", pngCompressedBytes.length, Arrays.toString(pngCompressedBytes));


      Mat finalDepth16UC1 = new Mat(128, 128, opencv_core.CV_16UC1);
      OpenCVTools.decompressDepthPNG(new BytePointer(pngCompressedBytes), finalDepth16UC1);

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
   public void testStoreAndLoadFloatArray()
   {
      HDF5Tools hdf5Tools = new HDF5Tools();
      hdf5ManagerWriter = new HDF5Manager("hdf5_test_float_array.hdf5", hdf5.H5F_ACC_TRUNC());

      Group writeGroup = hdf5ManagerWriter.createOrGetGroup("/test/bytes/");

      float[] floatArray = new float[]{12.3f, 32.1f, 43.1f, 32.43f};
      FloatPointer dataArray = new FloatPointer(floatArray);

      hdf5Tools.storeFloatArray2D(writeGroup, 0, dataArray, 1, floatArray.length);

      writeGroup._close();
      hdf5ManagerWriter.getFile()._close();

      hdf5ManagerReader = new HDF5Manager("hdf5_test_float_array.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.openOrGetGroup("/test/bytes/");

      float[] loadedFloats = hdf5Tools.loadFloatArray(readGroup, 0);

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
   public void testHDF5Attributes()
   {
      HDF5Tools hdf5Tools = new HDF5Tools();
      hdf5ManagerWriter = new HDF5Manager("test_attributes.hdf5", hdf5.H5F_ACC_TRUNC());
      Group writeGroup = hdf5ManagerWriter.createOrGetGroup("/test/bytes/");

      byte[] dataArray = new byte[40];
      for (int i = 0; i < dataArray.length; i++)
      {
         dataArray[i] = (byte) i;
      }
      hdf5Tools.storeByteArray(writeGroup, 0, dataArray, dataArray.length);

      // Create an attribute for storing size of the byte array
      long[] dims = {1};
      DataType dataType = new DataType(PredType.NATIVE_INT32());
      DataSpace dataSpace = new DataSpace(1, dims);
      Attribute attribute = writeGroup.createAttribute("Size", dataType, dataSpace);
      IntPointer pointer = new IntPointer(1);
      pointer.put(0, dataArray.length);
      attribute.write(dataType, pointer);
      attribute._close();
      writeGroup._close();
      hdf5ManagerWriter.getFile()._close();

      hdf5ManagerReader = new HDF5Manager("test_attributes.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.openOrGetGroup("/test/bytes/");

      Attribute sizeAttribute = readGroup.openAttribute("Size");
      IntPointer sizeAttributePointer = new IntPointer(1);
      sizeAttribute.read(dataType, sizeAttributePointer);
      sizeAttribute._close();

      assertEquals(dataArray.length, sizeAttributePointer.get(0));

      hdf5ManagerWriter.closeFile();
      hdf5ManagerReader.closeFile();
   }

   @Test
   public void testHDF5ToolsAttributesAPI()
   {
      HDF5Tools hdf5Tools = new HDF5Tools();
      hdf5ManagerWriter = new HDF5Manager("test_attributes_api.hdf5", hdf5.H5F_ACC_TRUNC());
      Group writeGroup = hdf5ManagerWriter.createOrGetGroup("/test/bytes/");

      byte[] dataArray = new byte[40];
      for (int i = 0; i < dataArray.length; i++)
      {
         dataArray[i] = (byte) i;
      }
      hdf5Tools.storeByteArray(writeGroup, 0, dataArray, dataArray.length);

      hdf5Tools.writeIntAttribute(writeGroup, "Int", 12345);
      hdf5Tools.writeLongAttribute(writeGroup, "Long", 123456);
      hdf5Tools.writeFloatAttribute(writeGroup, "Float", 0.12345f);
      hdf5Tools.writeDoubleAttribute(writeGroup, "Double", 0.123456);
      hdf5Tools.writeStringAttribute(writeGroup, "String", "Hello World");

      writeGroup._close();
      hdf5ManagerWriter.getFile()._close();

      hdf5ManagerReader = new HDF5Manager("test_attributes_api.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.openOrGetGroup("/test/bytes/");

      int value = hdf5Tools.readIntAttribute(readGroup, "Int");
      long longValue = hdf5Tools.readLongAttribute(readGroup, "Long");
      float floatValue = hdf5Tools.readFloatAttribute(readGroup, "Float");
      double doubleValue = hdf5Tools.readDoubleAttribute(readGroup, "Double");
      String stringValue = hdf5Tools.readStringAttribute(readGroup, "String");

      assertEquals(12345, value);
      assertEquals(123456, longValue);
      assertEquals(0.12345f, floatValue, 1e-10);
      assertEquals(0.123456, doubleValue, 1e-10);
      assertEquals("Hello World", stringValue);

      hdf5ManagerWriter.closeFile();
      hdf5ManagerReader.closeFile();
   }

   @Disabled
   @Test
   public void testPerceptionDataLoggerStoreFloats()
   {
      PerceptionDataLogger perceptionDataLogger = new PerceptionDataLogger();
      perceptionDataLogger.openLogFile("test_pointer_api.hdf5");

      String pointChannelName = "/test/pointer/points/";
      String quaternionChannelName = "/test/pointer/quaternions/";

      perceptionDataLogger.addFloatChannel(pointChannelName, 3, PerceptionLoggerConstants.DEFAULT_BLOCK_SIZE);
      perceptionDataLogger.addFloatChannel(quaternionChannelName, 4, PerceptionLoggerConstants.DEFAULT_BLOCK_SIZE);

      perceptionDataLogger.storeFloats(pointChannelName, new Point3D(0, 1, 2));
      perceptionDataLogger.storeFloats(quaternionChannelName, new Quaternion(0, 1, 2, 3));
      perceptionDataLogger.closeLogFile();

      PerceptionDataLoader perceptionDataLoader = new PerceptionDataLoader();
      perceptionDataLoader.openLogFile("test_pointer_api.hdf5");
      perceptionDataLoader.addFloatChannel(pointChannelName, 3, PerceptionLoggerConstants.DEFAULT_BLOCK_SIZE);
      perceptionDataLoader.addFloatChannel(quaternionChannelName, 4, PerceptionLoggerConstants.DEFAULT_BLOCK_SIZE);

      ArrayList<Point3D> points = new ArrayList<>();
      ArrayList<Quaternion> quaternions = new ArrayList<>();
      perceptionDataLoader.loadPoint3DList(pointChannelName, points);
      perceptionDataLoader.loadQuaternionList(quaternionChannelName, quaternions);
      perceptionDataLoader.closeLogFile();

      LogTools.info("Total Points: {}", points.size());
      LogTools.info("Total Quaternions: {}", quaternions.size());

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(0, 1, 2), points.get(0), 1e-7);
      EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(new Quaternion(0, 1, 2, 3), quaternions.get(0), 1e-7);
   }

   @Test
   @Disabled
   public void testCompressedDepthLoggingJPG()
   {
//      hdf5ManagerWriter = new HDF5Manager("hdf5_test.hdf5", hdf5.H5F_ACC_TRUNC());

      Mat depth = new Mat(128, 128, opencv_core.CV_16UC1);
      depth.put(new Scalar(12345));

      BytePointer compressedDepthPointer = new BytePointer();
      OpenCVTools.compressDepthJPG(depth, compressedDepthPointer);

      byte[] dataArray = new byte[compressedDepthPointer.asBuffer().remaining()];
      compressedDepthPointer.asBuffer().get(dataArray, 0, dataArray.length);

//      LogTools.info("Raw Size: {}, Compressed Size: {}", depth.rows() * depth.cols() * 4, dataArray.length);
//
//      Group writeGroup = hdf5ManagerWriter.getGroup("/test/bytes/");
//
//      LogTools.info("PNG Stored: [{}] -> {}", dataArray.length, Arrays.toString(dataArray));
//
//      hdf5Tools.storeByteArray(writeGroup, 0, dataArray, dataArray.length);
//
//      writeGroup._close();
//      hdf5ManagerWriter.getFile()._close();
//
//      hdf5ManagerReader = new HDF5Manager("hdf5_test.hdf5", hdf5.H5F_ACC_RDONLY());
//      Group readGroup = hdf5ManagerReader.openGroup("/test/bytes/");
//
//      byte[] pngCompressedBytes = hdf5Tools.loadByteArray(readGroup, 0);

//      LogTools.info("PNG Loaded: [{}] -> {}", dataArray.length, Arrays.toString(dataArray));


      Mat finalDepth16UC1 = new Mat(128, 128, opencv_core.CV_16UC1);
      OpenCVTools.decompressJPG(dataArray, finalDepth16UC1);

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
      OpenCVTools.compressDepthJPG(depth, compressedDepthPointer);

      byte[] dataArray = new byte[compressedDepthPointer.asBuffer().remaining()];
      compressedDepthPointer.asBuffer().get(dataArray, 0, dataArray.length);

//      LogTools.info("PNG Loaded: [{}] -> {}", dataArray.length, Arrays.toString(dataArray));

      Mat finalDepth16UC1 = new Mat(128, 128, opencv_core.CV_16UC1);
      OpenCVTools.decompressJPG(dataArray, finalDepth16UC1);

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

   @Test
   @Disabled
   public void testLoggingIntArray()
   {
      HDF5Tools hdf5Tools = new HDF5Tools();
      hdf5ManagerWriter = new HDF5Manager("hdf5_test_int_array.hdf5", hdf5.H5F_ACC_TRUNC());
      Group writeGroup = hdf5ManagerWriter.createOrGetGroup("/test/ints/");

      int[] dataArray = {0, 255, 1, 3, 4, 42, 153};

      hdf5Tools.storeIntArray(writeGroup, 0, dataArray, dataArray.length);

      writeGroup._close();
      hdf5ManagerWriter.getFile()._close();

      hdf5ManagerReader = new HDF5Manager("hdf5_test_int_array.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.createOrGetGroup("/test/ints/");
      int[] outputArray = hdf5Tools.loadIntArray(readGroup, 0);

      assertEquals(dataArray.length, 7);
      assertEquals(outputArray.length, 7);

      for (int i = 0; i < dataArray.length; i++)
      {
         assertEquals(dataArray[i], outputArray[i]);
      }
      hdf5ManagerWriter.closeFile();
      hdf5ManagerReader.closeFile();
   }
}
