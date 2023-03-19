package us.ihmc.perception.logging;

import org.bytedeco.hdf5.*;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.*;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;

import java.io.File;
import java.nio.ByteOrder;
import java.util.Arrays;

public class HDF5Test
{
   static
   {
      LogTools.info("Native byte order: {}", ByteOrder.nativeOrder());
   }

   @Test
   public void testInts()
   {
      File file = new File("NativeInts.hdf5");

      String filePath = file.getAbsolutePath();
      H5File h5File = new H5File(filePath, hdf5.H5F_ACC_TRUNC);
      String datasetId = "ints";

      int[] writeData = new int[] { -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, Integer.MAX_VALUE, Integer.MIN_VALUE };
      IntPointer dataPointer = new IntPointer(writeData);

      int rank = 1;
      long[] dimensions = { dataPointer.limit() };
      DataSpace fileSpace = new DataSpace(rank, dimensions);
      DataType dataType = new DataType(PredType.NATIVE_INT());
      DataSet dataSet = h5File.createDataSet(datasetId, dataType, fileSpace);

      dataSet.write(dataPointer, dataType);

      dataSet._close();
      fileSpace._close();
      h5File._close();

      h5File = new H5File(filePath, hdf5.H5F_ACC_RDONLY);
      dataSet = h5File.openDataSet(datasetId);

      dataPointer = new IntPointer(writeData.length);
      dataSet.read(dataPointer, dataType);

      int[] readData = new int[writeData.length];
      dataPointer.get(readData);

      dataSet._close();
      h5File._close();

      LogTools.info("Wrote: {}", Arrays.toString(writeData));
      LogTools.info("Read:  {}", Arrays.toString(readData));
      Assertions.assertArrayEquals(writeData, readData);
   }

   @Test
   public void testFloats()
   {
      File file = new File("NativeFloats.hdf5");

      String filePath = file.getAbsolutePath();
      H5File h5File = new H5File(filePath, hdf5.H5F_ACC_TRUNC);
      String datasetId = "floats";

      float[] writeData = new float[] { -2.0f, -1.1f, 0.0f, 1.0f, 2.22f, 3.2f, 4.8f, 5.5f, 6.0f, 7.99f, 8.1f, 9.3f,
                                        Float.MAX_VALUE, Float.MIN_VALUE, Float.NaN, Float.NEGATIVE_INFINITY, Float.POSITIVE_INFINITY };
      FloatPointer dataPointer = new FloatPointer(writeData);

      int rank = 1;
      long[] dimensions = { dataPointer.limit() };
      DataSpace fileSpace = new DataSpace(rank, dimensions);
      DataType dataType = new DataType(PredType.NATIVE_FLOAT());
      DataSet dataSet = h5File.createDataSet(datasetId, dataType, fileSpace);

      dataSet.write(dataPointer, dataType);

      dataSet._close();
      fileSpace._close();
      h5File._close();

      h5File = new H5File(filePath, hdf5.H5F_ACC_RDONLY);
      dataSet = h5File.openDataSet(datasetId);

      dataPointer = new FloatPointer(writeData.length);
      dataSet.read(dataPointer, dataType);

      float[] readData = new float[writeData.length];
      dataPointer.get(readData);

      dataSet._close();
      h5File._close();

      LogTools.info("Wrote: {}", Arrays.toString(writeData));
      LogTools.info("Read:  {}", Arrays.toString(readData));
      Assertions.assertArrayEquals(writeData, readData);
   }

   @Test
   public void testDoubles()
   {
      File file = new File("NativeDoubles.hdf5");

      String filePath = file.getAbsolutePath();
      H5File h5File = new H5File(filePath, hdf5.H5F_ACC_TRUNC);
      String datasetId = "doubles";

      double[] writeData = new double[] { -2.0, -1.1, 0.0, 1.0, 2.22, 3.2, 4.8, 5.5, 6.0, 7.99, 8.1, 9.3,
                                        Double.MAX_VALUE, Double.MIN_VALUE, Double.NaN, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY };
      DoublePointer dataPointer = new DoublePointer(writeData);

      int rank = 1;
      long[] dimensions = { dataPointer.limit() };
      DataSpace fileSpace = new DataSpace(rank, dimensions);
      DataType dataType = new DataType(PredType.NATIVE_DOUBLE());
      DataSet dataSet = h5File.createDataSet(datasetId, dataType, fileSpace);

      dataSet.write(dataPointer, dataType);

      dataSet._close();
      fileSpace._close();
      h5File._close();

      h5File = new H5File(filePath, hdf5.H5F_ACC_RDONLY);
      dataSet = h5File.openDataSet(datasetId);

      dataPointer = new DoublePointer(writeData.length);
      dataSet.read(dataPointer, dataType);

      double[] readData = new double[writeData.length];
      dataPointer.get(readData);

      dataSet._close();
      h5File._close();

      LogTools.info("Wrote: {}", Arrays.toString(writeData));
      LogTools.info("Read:  {}", Arrays.toString(readData));
      Assertions.assertArrayEquals(writeData, readData);
   }

   @Test
   public void testChars()
   {
      File file = new File("NativeChars.hdf5");

      String filePath = file.getAbsolutePath();
      H5File h5File = new H5File(filePath, hdf5.H5F_ACC_TRUNC);
      String datasetId = "chars";

      char[] writeData = new char[] { 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 0,
                                      Character.MAX_VALUE, Character.MIN_VALUE, Character.MAX_HIGH_SURROGATE, Character.MIN_HIGH_SURROGATE};
      CharPointer dataPointer = new CharPointer(writeData);

      int rank = 1;
      long[] dimensions = { dataPointer.limit() };
      DataSpace fileSpace = new DataSpace(rank, dimensions);
      DataType dataType = new DataType(PredType.NATIVE_B16()); // A Java char is 2 bytes
      DataSet dataSet = h5File.createDataSet(datasetId, dataType, fileSpace);

      dataSet.write(dataPointer, dataType);

      dataSet._close();
      fileSpace._close();
      h5File._close();

      h5File = new H5File(filePath, hdf5.H5F_ACC_RDONLY);
      dataSet = h5File.openDataSet(datasetId);

      dataPointer = new CharPointer(writeData.length);
      dataSet.read(dataPointer, dataType);

      char[] readData = new char[writeData.length];
      dataPointer.get(readData);

      dataSet._close();
      h5File._close();

      LogTools.info("Wrote: {}", Arrays.toString(writeData));
      LogTools.info("Read:  {}", Arrays.toString(readData));
      Assertions.assertArrayEquals(writeData, readData);
   }

   @Test
   public void testShorts()
   {
      File file = new File("NativeShorts.hdf5");

      String filePath = file.getAbsolutePath();
      H5File h5File = new H5File(filePath, hdf5.H5F_ACC_TRUNC);
      String datasetId = "shorts";

      short[] writeData = new short[] { -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, Short.MAX_VALUE, Short.MIN_VALUE};
      ShortPointer dataPointer = new ShortPointer(writeData);

      int rank = 1;
      long[] dimensions = { dataPointer.limit() };
      DataSpace fileSpace = new DataSpace(rank, dimensions);
      DataType dataType = new DataType(PredType.NATIVE_B16()); // A Java short is 2 bytes
      DataSet dataSet = h5File.createDataSet(datasetId, dataType, fileSpace);

      dataSet.write(dataPointer, dataType);

      dataSet._close();
      fileSpace._close();
      h5File._close();

      h5File = new H5File(filePath, hdf5.H5F_ACC_RDONLY);
      dataSet = h5File.openDataSet(datasetId);

      dataPointer = new ShortPointer(writeData.length);
      dataSet.read(dataPointer, dataType);

      short[] readData = new short[writeData.length];
      dataPointer.get(readData);

      dataSet._close();
      h5File._close();

      LogTools.info("Wrote: {}", Arrays.toString(writeData));
      LogTools.info("Read:  {}", Arrays.toString(readData));
      Assertions.assertArrayEquals(writeData, readData);
   }

   @Test
   public void testBytesWithoutZeros()
   {
      File file = new File("NativeBytesWithoutZeros.hdf5");

      String filePath = file.getAbsolutePath();
      H5File h5File = new H5File(filePath, hdf5.H5F_ACC_TRUNC);
      String datasetId = "bytes";

      byte[] writeData = new byte[] { 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'f',
                                      -2, -1, 1, 2, 3, 4, Byte.MIN_VALUE, Byte.MAX_VALUE };
      BytePointer dataPointer = new BytePointer(writeData);

      int rank = 1;
      long[] dimensions = { dataPointer.limit() };
      DataSpace fileSpace = new DataSpace(rank, dimensions);
      DataType dataType = new DataType(PredType.NATIVE_B8());
      DataSet dataSet = h5File.createDataSet(datasetId, dataType, fileSpace);

      dataSet.write((Pointer) dataPointer, dataType);

      dataSet._close();
      fileSpace._close();
      h5File._close();

      h5File = new H5File(filePath, hdf5.H5F_ACC_RDONLY);
      dataSet = h5File.openDataSet(datasetId);

      dataPointer = new BytePointer(writeData.length);
      dataSet.read((Pointer) dataPointer, dataType);

      byte[] readData = new byte[(int) dataPointer.limit()];
      dataPointer.get(readData);

      dataSet._close();
      h5File._close();

      LogTools.info("Wrote: {}", Arrays.toString(writeData));
      LogTools.info("Read:  {}", Arrays.toString(readData));
      Assertions.assertArrayEquals(writeData, readData);
   }

   @Test
   public void testBytesWithZeros()
   {
      File file = new File("NativeBytesWithZeros.hdf5");

      String filePath = file.getAbsolutePath();
      H5File h5File = new H5File(filePath, hdf5.H5F_ACC_TRUNC);
      String datasetId = "bytes";

      // There is a 0 in here that causes the problem
      //byte[] writeData = new byte[] { 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'f',
      //                                0, -2, -1, 1, 0, 2, 3, 4, Byte.MIN_VALUE, Byte.MAX_VALUE };

      byte[] writeData = new byte[10000];

      for (int i = 0; i < writeData.length; i++)
      {
         writeData[i] = (byte) i;
      }

      BytePointer dataPointer = new BytePointer(writeData);

      byte[] beforeWriteData = new byte[(int) dataPointer.limit()];
      dataPointer.get(beforeWriteData);
      LogTools.info("Before: {}", Arrays.toString(beforeWriteData));
      LogTools.info("Writing {} bytes", writeData.length);

      int rank = 1;
      long[] dimensions = { dataPointer.limit() };
      DataSpace fileSpace = new DataSpace(rank, dimensions);
      DataType dataType = new DataType(PredType.NATIVE_B8());
      DataSet dataSet = h5File.createDataSet(datasetId, dataType, fileSpace);

      dataSet.write((Pointer) dataPointer, dataType);
      LogTools.info("Wrote:  {}", Arrays.toString(writeData));

      dataSet._close();
      fileSpace._close();
      h5File._close();

      h5File = new H5File(filePath, hdf5.H5F_ACC_RDONLY);
      dataSet = h5File.openDataSet(datasetId);

      // Casting to Pointer required to avoid calling the string method
      // Until this is fixed
      // See https://github.com/bytedeco/javacpp-presets/issues/1311
      dataPointer = new BytePointer(writeData.length);
      dataSet.read((Pointer) dataPointer, dataType);
      LogTools.info("Read    {} bytes", dataPointer.limit());
      byte[] readData = new byte[(int) dataPointer.limit()];
      dataPointer.get(readData);
      LogTools.info("Read:   {}", Arrays.toString(readData));

      dataSet._close();
      h5File._close();

      Assertions.assertArrayEquals(writeData, readData);
   }

   /**
    * Make sure we can successively close and reopen an HDF5 file
    */
   @Test
   public void testCloseAndReopenInts()
   {
      for (int i = 0; i < 5; i++)
      {
         testInts();
      }
   }
}
