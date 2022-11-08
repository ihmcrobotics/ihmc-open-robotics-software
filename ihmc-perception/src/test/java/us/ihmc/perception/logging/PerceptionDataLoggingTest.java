package us.ihmc.perception.logging;

import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.global.hdf5;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.Arrays;

import static us.ihmc.robotics.Assert.assertEquals;

public class PerceptionDataLoggingTest
{
   private HDF5Manager hdf5ManagerReader;
   private HDF5Manager hdf5ManagerWriter;

   @Test
   public void testLoggingByteArray()
   {
      hdf5ManagerWriter = new HDF5Manager("/home/quantum/Workspace/Data/Sensor_Logs/hdf5_test.hdf5", hdf5.H5F_ACC_TRUNC());
      Group writeGroup = hdf5ManagerWriter.getGroup("/test/bytes/");

      byte[] dataArray = {(byte) 0, (byte) 255, (byte) 1, (byte) 3, (byte) 4, (byte) 42, (byte) 153, (byte) 0};

      ByteBuffer buffer = ByteBuffer.wrap(dataArray);
      IntBuffer intBuffer = buffer.asIntBuffer();
      int[] array = new int[dataArray.length / Integer.BYTES];
      intBuffer.get(array);

      HDF5Tools.storeIntArray(writeGroup, 0, array, array.length);

      writeGroup.close();
      hdf5ManagerWriter.getFile().close();

      hdf5ManagerReader = new HDF5Manager("/home/quantum/Workspace/Data/Sensor_Logs/hdf5_test.hdf5", hdf5.H5F_ACC_RDONLY());
      Group readGroup = hdf5ManagerReader.openGroup("/test/bytes/");
      int[] outputIntArray = HDF5Tools.loadIntArray(readGroup, 0);

      byte[] outputArray = new byte[outputIntArray.length * Integer.BYTES];
      ByteBuffer byteBuffer = ByteBuffer.wrap(outputArray);
      IntBuffer outputIntBuffer = byteBuffer.asIntBuffer();
      outputIntBuffer.put(outputIntArray);

      for (int i = 0; i < dataArray.length; i++)
      {
         assertEquals(dataArray[i], outputArray[i]);
      }
      hdf5ManagerReader.getFile().close();
   }

   @Test
   public void testLoggingLargeByteArray()
   {
      hdf5ManagerWriter = new HDF5Manager("/home/quantum/Workspace/Data/Sensor_Logs/hdf5_test.hdf5", hdf5.H5F_ACC_TRUNC());
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

      hdf5ManagerReader = new HDF5Manager("/home/quantum/Workspace/Data/Sensor_Logs/hdf5_test.hdf5", hdf5.H5F_ACC_RDONLY());
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
   public void testLoggingIntArray()
   {
      hdf5ManagerWriter = new HDF5Manager("/home/quantum/Workspace/Data/Sensor_Logs/hdf5_test.hdf5", hdf5.H5F_ACC_TRUNC());
      Group writeGroup = hdf5ManagerWriter.getGroup("/test/ints/");

      int[] dataArray = {0, 255, 1, 3, 4, 42, 153};

      HDF5Tools.storeIntArray(writeGroup, 0, dataArray, dataArray.length);

      writeGroup.close();
      hdf5ManagerWriter.getFile().close();

      hdf5ManagerReader = new HDF5Manager("/home/quantum/Workspace/Data/Sensor_Logs/hdf5_test.hdf5", hdf5.H5F_ACC_RDONLY());
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
}
