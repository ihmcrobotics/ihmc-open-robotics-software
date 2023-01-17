package us.ihmc.perception.logging;

import org.bytedeco.hdf5.*;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.*;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

/**
 * We can't initialize an HDF5 file from resource currently, so we can't run this on CI at all.
 */
@Disabled
public class HDF5Test
{
   static
   {
      LogTools.info("Native byte order: {}", ByteOrder.nativeOrder());
   }

   @Test
   public void testInts()
   {
      WorkspaceFile file = getFile("NativeInts");

      String filePath = file.getFilePath().toString();
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

      dataSet.close();
      fileSpace.close();
      h5File.close();

      h5File = new H5File(filePath, hdf5.H5F_ACC_RDONLY);
      dataSet = h5File.openDataSet(datasetId);

      dataPointer = new IntPointer(writeData.length);
      dataSet.read(dataPointer, dataType);

      int[] readData = new int[writeData.length];
      dataPointer.get(readData);

      dataSet.close();
      h5File.close();

      LogTools.info("Wrote: {}", Arrays.toString(writeData));
      LogTools.info("Read:  {}", Arrays.toString(readData));
      Assertions.assertArrayEquals(writeData, readData);
   }

   @Test
   public void testFloats()
   {
      WorkspaceFile file = getFile("NativeFloats");

      String filePath = file.getFilePath().toString();
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

      dataSet.close();
      fileSpace.close();
      h5File.close();

      h5File = new H5File(filePath, hdf5.H5F_ACC_RDONLY);
      dataSet = h5File.openDataSet(datasetId);

      dataPointer = new FloatPointer(writeData.length);
      dataSet.read(dataPointer, dataType);

      float[] readData = new float[writeData.length];
      dataPointer.get(readData);

      dataSet.close();
      h5File.close();

      LogTools.info("Wrote: {}", Arrays.toString(writeData));
      LogTools.info("Read:  {}", Arrays.toString(readData));
      Assertions.assertArrayEquals(writeData, readData);
   }

   @Test
   public void testDoubles()
   {
      WorkspaceFile file = getFile("NativeDoubles");

      String filePath = file.getFilePath().toString();
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

      dataSet.close();
      fileSpace.close();
      h5File.close();

      h5File = new H5File(filePath, hdf5.H5F_ACC_RDONLY);
      dataSet = h5File.openDataSet(datasetId);

      dataPointer = new DoublePointer(writeData.length);
      dataSet.read(dataPointer, dataType);

      double[] readData = new double[writeData.length];
      dataPointer.get(readData);

      dataSet.close();
      h5File.close();

      LogTools.info("Wrote: {}", Arrays.toString(writeData));
      LogTools.info("Read:  {}", Arrays.toString(readData));
      Assertions.assertArrayEquals(writeData, readData);
   }

   @Test
   public void testChars()
   {
      WorkspaceFile file = getFile("NativeChars");

      String filePath = file.getFilePath().toString();
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

      dataSet.close();
      fileSpace.close();
      h5File.close();

      h5File = new H5File(filePath, hdf5.H5F_ACC_RDONLY);
      dataSet = h5File.openDataSet(datasetId);

      dataPointer = new CharPointer(writeData.length);
      dataSet.read(dataPointer, dataType);

      char[] readData = new char[writeData.length];
      dataPointer.get(readData);

      dataSet.close();
      h5File.close();

      LogTools.info("Wrote: {}", Arrays.toString(writeData));
      LogTools.info("Read:  {}", Arrays.toString(readData));
      Assertions.assertArrayEquals(writeData, readData);
   }

   @Test
   public void testShorts()
   {
      WorkspaceFile file = getFile("NativeShorts");

      String filePath = file.getFilePath().toString();
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

      dataSet.close();
      fileSpace.close();
      h5File.close();

      h5File = new H5File(filePath, hdf5.H5F_ACC_RDONLY);
      dataSet = h5File.openDataSet(datasetId);

      dataPointer = new ShortPointer(writeData.length);
      dataSet.read(dataPointer, dataType);

      short[] readData = new short[writeData.length];
      dataPointer.get(readData);

      dataSet.close();
      h5File.close();

      LogTools.info("Wrote: {}", Arrays.toString(writeData));
      LogTools.info("Read:  {}", Arrays.toString(readData));
      Assertions.assertArrayEquals(writeData, readData);
   }

   @Test
   public void testBytesWithoutZeros()
   {
      WorkspaceFile file = getFile("NativeBytesWithoutZeros");

      String filePath = file.getFilePath().toString();
      H5File h5File = new H5File(filePath, hdf5.H5F_ACC_TRUNC);
      String datasetId = "bytes";

      byte[] writeData = new byte[] { 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'f',
                                      -0x02, -0x01, 0x01, 0x02, 0x03, 0x04, 0x05, Byte.MIN_VALUE, Byte.MAX_VALUE };
      BytePointer dataPointer = new BytePointer(writeData);

      int rank = 1;
      long[] dimensions = { dataPointer.limit() };
      DataSpace fileSpace = new DataSpace(rank, dimensions);
      DataType dataType = new DataType(PredType.NATIVE_B8());
      DataSet dataSet = h5File.createDataSet(datasetId, dataType, fileSpace);

      dataSet.write(dataPointer, dataType);

      dataSet.close();
      fileSpace.close();
      h5File.close();

      h5File = new H5File(filePath, hdf5.H5F_ACC_RDONLY);
      dataSet = h5File.openDataSet(datasetId);

      dataPointer = new BytePointer(writeData.length);
      dataSet.read(dataPointer, dataType);

      byte[] readData = new byte[(int) dataPointer.limit()];
      dataPointer.get(readData);

      dataSet.close();
      h5File.close();

      LogTools.info("Wrote: {}", Arrays.toString(writeData));
      LogTools.info("Read:  {}", Arrays.toString(readData));
      Assertions.assertArrayEquals(writeData, readData);
   }

   @Test
   public void testBytesWithZeros()
   {
      WorkspaceFile file = getFile("NativeBytesWithZeros");

      String filePath = file.getFilePath().toString();
      H5File h5File = new H5File(filePath, hdf5.H5F_ACC_TRUNC);
      String datasetId = "bytes";

      // There is a 0 in here that causes the problem
      byte[] writeData = new byte[] { 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'f',
                                      0, -2, -1, 1, 0, 2, 3, 4, Byte.MIN_VALUE, Byte.MAX_VALUE };
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

      dataSet.write(dataPointer, dataType);
      LogTools.info("Wrote:  {}", Arrays.toString(writeData));

      dataSet.close();
      fileSpace.close();
      h5File.close();

      h5File = new H5File(filePath, hdf5.H5F_ACC_RDONLY);
      dataSet = h5File.openDataSet(datasetId);

      ShortPointer shortPointer = new ShortPointer(writeData.length);
      dataSet.read(shortPointer, dataType);
      LogTools.info("Read    {} shorts", shortPointer.limit());
      ByteBuffer byteBuffer = shortPointer.asByteBuffer();
      byte[] shortReadData = new byte[(int) writeData.length];
      byteBuffer.get(shortReadData);
      LogTools.info("Read:   {}", Arrays.toString(shortReadData));

      dataPointer = new BytePointer(writeData.length);
      dataSet.read(dataPointer, dataType);
      LogTools.info("Read    {} bytes", dataPointer.limit());
      byte[] readData = new byte[(int) dataPointer.limit()];
      dataPointer.get(readData);
      LogTools.info("Read:   {}", Arrays.toString(readData));

      dataSet.close();
      h5File.close();

      Assertions.assertArrayEquals(writeData, shortReadData);
      Assertions.assertArrayEquals(writeData, readData);
   }

   private static WorkspaceFile getFile(String name)
   {
      return new WorkspaceFile(getDirectory(), name + HDF5Tools.HDF5_FILE_EXTENSION);
   }

   private static WorkspaceDirectory getDirectory()
   {
      WorkspaceDirectory directory = new WorkspaceDirectory("ihmc-open-robotics-software", "ihmc-perception/src/test/resources", HDF5Test.class);
      FileTools.ensureDirectoryExists(directory.getDirectoryPath(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      return directory;
   }
}
