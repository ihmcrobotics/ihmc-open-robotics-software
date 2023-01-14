package us.ihmc.perception.logging;

import org.bytedeco.hdf5.*;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class HDF5Test
{

   /**
    * File can't be written on CI servers. Must be done manually.
    */
   public static void writeTestFile()
   {
      WorkspaceDirectory directory = getDirectory();
      WorkspaceFile file = new WorkspaceFile(directory, "ByteQuartet" + HDF5Tools.HDF5_FILE_EXTENSION);
      FileTools.ensureDirectoryExists(directory.getDirectoryPath(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

      printTypeInfo();

      String filePath = file.getFilePath().toString();
      H5File h5File = new H5File(filePath, hdf5.H5F_ACC_TRUNC);
      Group group = h5File.createGroup("quartet");

      BytePointer bytePointer = new BytePointer(4 * Integer.BYTES);
      bytePointer.put((byte) 0);
      bytePointer.put((byte) 0);
      bytePointer.put((byte) 0);
      bytePointer.put((byte) 0);
      bytePointer.position(0);

//      DSetCreatPropList dSetCreatPropList = new DSetCreatPropList();
//      dSetCreatPropList.setFillValue(PredType.NATIVE_B8(), new BytePointer(0));

      int rank = 1;
      long[] dimensions = { bytePointer.limit() };
      DataSpace fileSpace = new DataSpace(rank, dimensions);

      DataType dataType = new DataType(PredType.NATIVE_B8());
      int index = 0;
      DataSet dataSet = group.createDataSet(String.valueOf(index), dataType, fileSpace);

//      fileSpace.selectElements(hdf5.H5S_SELECT_SET, 4, new long[] { 0 });
//
//      DataSpace memorySpace = new DataSpace(rank);
//      memorySpace.selectElements(hdf5.H5S_SELECT_SET, 4, new long[] { 0 });
//
//      DSetMemXferPropList dSetMemXferPropList = new DSetMemXferPropList(hdf5.H5P_DEFAULT());
//      dataSet.write(bytePointer, dataType, memorySpace, fileSpace, dSetMemXferPropList);


      dataSet.write(bytePointer, dataType);

      dataSet.close();
      fileSpace.close();

      group.close();
      h5File.close();
   }

   @Test
   public void testReadingBytes()
   {
      WorkspaceDirectory directory = getDirectory();
      WorkspaceFile file = new WorkspaceFile(directory, "ByteQuartet" + HDF5Tools.HDF5_FILE_EXTENSION);

      printTypeInfo();

      String filePath = file.getFilePath().toString();
      H5File h5File = new H5File(filePath, hdf5.H5F_ACC_RDONLY);
      Group group = h5File.openGroup("quartet");


      BytePointer intPointer = new BytePointer(40 * Integer.BYTES);

      LogTools.info(intPointer);
      LogTools.info(intPointer.get(0));
      LogTools.info(intPointer.get(1));
      LogTools.info(intPointer.get(2));
      LogTools.info(intPointer.get(3));

      int index = 0;
      DataSet dataSet = group.openDataSet(String.valueOf(index));
//      DataSpace dataSpace = dataSet.getSpace();
//      dataSpace.selectElements(hdf5.H5S_SELECT_SET, 4, new long[] { 0 });
//
//      int rank = 1;
//      DataSpace memorySpace = new DataSpace(rank);
//      memorySpace.selectElements(hdf5.H5S_SELECT_SET, 4, new long[] { 0 });

      DataType dataType = new DataType(PredType.NATIVE_B8());
//      DSetMemXferPropList dSetMemXferPropList = new DSetMemXferPropList(hdf5.H5P_DEFAULT());
//      dataSet.read(bytePointer, dataType, memorySpace, dataSpace, dSetMemXferPropList);

      dataSet.read(intPointer, dataType);

      LogTools.info(intPointer);
      LogTools.info(intPointer.get(0));
      LogTools.info(intPointer.get(1));
      LogTools.info(intPointer.get(2));
      LogTools.info(intPointer.get(3));


      dataSet.close();

      group.close();
      h5File.close();
   }

   private static void printTypeInfo()
   {
      PredType nativeB8 = PredType.NATIVE_B8();
      ByteOrder nativeByteOrder = ByteOrder.nativeOrder();
      LogTools.info("nativeB8: {}", nativeB8);
      LogTools.info("STD_B8BE: {}", PredType.STD_B8BE());
      LogTools.info("STD_B8LE: {}", PredType.STD_B8LE());
      LogTools.info("Native byte order: {}", nativeByteOrder);
   }

   private static WorkspaceDirectory getDirectory()
   {
      return new WorkspaceDirectory("ihmc-open-robotics-software", "ihmc-perception/src/test/resources", HDF5Test.class);
   }

   public static void main(String[] args)
   {
      HDF5Test.writeTestFile();
   }
}
