package us.ihmc.perception.logging;

import org.bytedeco.hdf5.*;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

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
      IntPointer intsPointer = new IntPointer(writeData);

      int rank = 1;
      long[] dimensions = { intsPointer.limit() };
      DataSpace fileSpace = new DataSpace(rank, dimensions);
      DataType dataType = new DataType(PredType.NATIVE_INT());
      DataSet dataSet = h5File.createDataSet("ints", dataType, fileSpace);

      dataSet.write(intsPointer, dataType);

      dataSet.close();
      fileSpace.close();
      h5File.close();

      h5File = new H5File(filePath, hdf5.H5F_ACC_RDONLY);
      dataSet = h5File.openDataSet(datasetId);

      IntPointer intPointer = new IntPointer(writeData.length);
      dataSet.read(intPointer, dataType);

      int[] readData = new int[writeData.length];
      intPointer.get(readData);

      dataSet.close();
      h5File.close();

      LogTools.info(Arrays.toString(writeData));
      LogTools.info(Arrays.toString(readData));
      Assertions.assertArrayEquals(writeData, readData);
   }

//   // Currently putting/pulling native bytes from HDF5 doesn't work
//   @Test
//   public void testNativeBytes()
//   {
//      WorkspaceFile file = getFile("NativeBytes");
//
//      String filePath = file.getFilePath().toString();
//      H5File h5File = new H5File(filePath, hdf5.H5F_ACC_TRUNC);
//      Group group = h5File.createGroup("quartet");
//
//      BytePointer bytePointer = new BytePointer(4 * Integer.BYTES);
//      bytePointer.put((byte) 0);
//      bytePointer.put((byte) 0);
//      bytePointer.put((byte) 0);
//      bytePointer.put((byte) 0);
//      bytePointer.position(0);
//
//      //      DSetCreatPropList dSetCreatPropList = new DSetCreatPropList();
//      //      dSetCreatPropList.setFillValue(PredType.NATIVE_B8(), new BytePointer(0));
//
//      int rank = 1;
//      long[] dimensions = { bytePointer.limit() };
//      DataSpace fileSpace = new DataSpace(rank, dimensions);
//
//      DataType dataType = new DataType(PredType.NATIVE_B8());
//      int index = 0;
//      DataSet dataSet = group.createDataSet(String.valueOf(index), dataType, fileSpace);
//
//      //      fileSpace.selectElements(hdf5.H5S_SELECT_SET, 4, new long[] { 0 });
//      //
//      //      DataSpace memorySpace = new DataSpace(rank);
//      //      memorySpace.selectElements(hdf5.H5S_SELECT_SET, 4, new long[] { 0 });
//      //
//      //      DSetMemXferPropList dSetMemXferPropList = new DSetMemXferPropList(hdf5.H5P_DEFAULT());
//      //      dataSet.write(bytePointer, dataType, memorySpace, fileSpace, dSetMemXferPropList);
//
//
//      dataSet.write(bytePointer, dataType);
//
//      dataSet.close();
//      fileSpace.close();
//
//      group.close();
//      h5File.close();
//
//
//      String filePath = file.getFilePath().toString();
//      H5File h5File = new H5File(filePath, hdf5.H5F_ACC_RDONLY);
//      Group group = h5File.openGroup("quartet");
//
//      BytePointer intPointer = new BytePointer(40 * Integer.BYTES);
//
//      LogTools.info(intPointer);
//      LogTools.info(intPointer.get(0));
//      LogTools.info(intPointer.get(1));
//      LogTools.info(intPointer.get(2));
//      LogTools.info(intPointer.get(3));
//
//      int index = 0;
//      DataSet dataSet = group.openDataSet(String.valueOf(index));
////      DataSpace dataSpace = dataSet.getSpace();
////      dataSpace.selectElements(hdf5.H5S_SELECT_SET, 4, new long[] { 0 });
////
////      int rank = 1;
////      DataSpace memorySpace = new DataSpace(rank);
////      memorySpace.selectElements(hdf5.H5S_SELECT_SET, 4, new long[] { 0 });
//
//      DataType dataType = new DataType(PredType.NATIVE_B8());
////      DSetMemXferPropList dSetMemXferPropList = new DSetMemXferPropList(hdf5.H5P_DEFAULT());
////      dataSet.read(bytePointer, dataType, memorySpace, dataSpace, dSetMemXferPropList);
//
//      dataSet.read(intPointer, dataType);
//
//      LogTools.info(intPointer);
//      LogTools.info(intPointer.get(0));
//      LogTools.info(intPointer.get(1));
//      LogTools.info(intPointer.get(2));
//      LogTools.info(intPointer.get(3));
//
//
//      dataSet.close();
//
//      group.close();
//      h5File.close();
//   }

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
