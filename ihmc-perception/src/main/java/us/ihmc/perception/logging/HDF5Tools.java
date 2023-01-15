package us.ihmc.perception.logging;

import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TLongArrayList;
import org.bytedeco.hdf5.*;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.*;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.Arrays;

public class HDF5Tools
{
   static final int NUMBER_OF_FIELDS_PER_POINT = 3;

   /**
    * Extracts the shape of a dataset and outputs the length along the requested dimension/axis (dim)
    *
    * @param dataSet The HDF5 dataset of which the shape is requested
    * @param dimension     The dimension or axis along which the length is requested
    * @return The length of the dataset along the requested dimension or axis
    */
   private static int extractShape(DataSet dataSet, int dimension)
   {
      DataSpace dataSpace = dataSet.getSpace();
      int dimensions = dataSpace.getSimpleExtentNdims();
      long[] shape = new long[dimensions];
      dataSpace.getSimpleExtentDims(shape);
      if (dimension < dimensions)
      {
         return (int) shape[dimension];
      }
      else
         return 0;
   }

   /**
    * Loads pointcloud from an HDF5 dataset into a RecyclingArrayList of 3D points.
    *
    * @param group           The HDF5 group where the requested pointcloud is stored
    * @param index           The index of the dataset within the requested group.
    * @param pointListToPack The recycling arraylist to be packed with the 3D points from pointcloud
    */
   public static void loadPointCloud(Group group, int index, RecyclingArrayList<Point3D32> pointListToPack, int rows, int cols)
   {
      DataSet dataset = group.openDataSet(String.valueOf(index));
      float[] pointsBuffer = new float[rows * cols * 3];
      FloatPointer p = new FloatPointer(pointsBuffer);
      dataset.read(p, PredType.NATIVE_FLOAT());
      p.get(pointsBuffer);

      pointListToPack.clear();
      for (int i = 0; i < pointsBuffer.length; i += 3)
      {
         Point3D32 point = pointListToPack.add();
         point.set(pointsBuffer[i], pointsBuffer[i + 1], pointsBuffer[i + 2]);
      }
   }

   /**
    * Loads a byte[] array from an HDF5 dataset within the requested group. Uses loadIntArray() as the backing method.
    *
    * @param group The HDF5 group where the requested pointcloud is stored
    * @param index The index of the dataset within the requested group.
    * @return The byte[] array with the data from HDF5 dataset.
    */
   public static byte[] loadByteArray(Group group, int index)
   {
      int[] loadedIntArray = HDF5Tools.loadIntArray(group, index);

      // Creates an empty byte array
      byte[] arrayToReturn = new byte[loadedIntArray.length * Integer.BYTES];

      // Creates an IntBuffer wrapper for the empty byte array
      ByteBuffer byteBuffer = ByteBuffer.wrap(arrayToReturn);
      IntBuffer loadedIntBuffer = byteBuffer.asIntBuffer();

      // Put the loaded int array into the wrapper for the empty byte array
      loadedIntBuffer.put(loadedIntArray);

      return arrayToReturn;
   }

   /**
    * Loads a int[] array from an HDF5 dataset within the requested group.
    *
    * @param group The HDF5 group where the requested pointcloud is stored
    * @param index The index of the dataset within the requested group.
    * @return The int[] array with the data from HDF5 dataset.
    */
   public static int[] loadIntArray(Group group, int index)
   {
      DataSet dataset = group.openDataSet(String.valueOf(index));

      int size = extractShape(dataset, 0);
      int[] intArray = new int[size];

      IntPointer intPointer = new IntPointer(intArray);
      dataset.read(intPointer, new DataType(PredType.NATIVE_INT()));
      intPointer.get(intArray, 0, intArray.length);

      dataset.close();

      return intArray;
   }

   /**
    * Stores a byte[] array into the requested HDF5 dataset. Uses storeIntArray() as the backing method.
    *
    * @param group The HDF5 group where the byte[] array data is to be stored
    * @param index The index of the dataset within the requested group.
    * @param data  The byte[] array to be stored into the HDF5 file.
    * @param size  Size of the relevant part of the data to be stored
    */
   public static void storeByteArray(Group group, long index, byte[] data, int size)
   {
      ByteBuffer buffer = ByteBuffer.wrap(data, 0, size);
      IntBuffer intBuffer = buffer.asIntBuffer();

      int intCount = (size / Integer.BYTES) + 1;
      int[] array = new int[intCount];
      intBuffer.get(array, 0, intCount - 1);

      HDF5Tools.storeIntArray(group, index, array, intCount);
   }

   /**
    * Stores an int[] array into the requested HDF5 dataset.
    *
    * @param group The HDF5 group where the int[] array data is to be stored
    * @param index The index of the dataset within the requested group.
    * @param data  The int[] array to be stored into the HDF5 file.
    * @param size  Size of the relevant part of the data to be stored
    */
   public static void storeIntArray(Group group, long index, int[] data, long size)
   {
      LogTools.info("Store Int Array: Index: {} Size: {}", index, size);
      long[] dimensions = {size};

      DataSpace dataSpace = new DataSpace(1, dimensions);
      DataSet dataset = group.createDataSet(String.valueOf(index), new DataType(PredType.NATIVE_INT()), dataSpace);

      dataset.write(new IntPointer(data), new DataType(PredType.NATIVE_INT()));

      dataSpace.close();
      dataset.close();
   }

   /**
    * Stores a pointcloud stored in an ArrayList of 3D points into an HDF5 dataset
    *
    * @param group           The HDF5 group where the requested pointcloud is stored
    * @param index           The index of the dataset within the requested group.
    * @param pointListToPack The final packed 3D point list with pointcloud
    */
   public static void storePointCloud(Group group, long index, ArrayList<Point3D> pointListToPack)
   {
      long[] dimensions = {pointListToPack.size(), NUMBER_OF_FIELDS_PER_POINT};
      DataSet dataset = group.createDataSet(String.valueOf(index), new DataType(PredType.NATIVE_FLOAT()), new DataSpace(2, dimensions));
      float[] buffer = new float[pointListToPack.size() * NUMBER_OF_FIELDS_PER_POINT];
      for (int i = 0; i < pointListToPack.size(); i++)
      {
         buffer[i * NUMBER_OF_FIELDS_PER_POINT] = pointListToPack.get(i).getX32();
         buffer[i * NUMBER_OF_FIELDS_PER_POINT + 1] = pointListToPack.get(i).getY32();
         buffer[i * NUMBER_OF_FIELDS_PER_POINT + 2] = pointListToPack.get(i).getZ32();
      }

      dataset.write(new FloatPointer(buffer), new DataType(PredType.NATIVE_FLOAT()));
   }

   /**
    * Stores a 2D long array passed as TLongArrayList into an HDF5 dataset
    *
    * @param group The HDF5 group where the requested is stored
    * @param index The index of the dataset within the requested group.
    * @param data  The long data for the 2D matrix to be stored
    * @param rows  Number of rows in the 2D matrix to be stored
    * @param cols  Number of columns in the 2D matrix to be stored
    */
   public static void storeLongArray2D(Group group, long index, TLongArrayList data, int rows, int cols)
   {
      // Log the timestamps buffer as separate dataset with same index
      long[] dimensions = {rows, cols};
      DataSet dataset = group.createDataSet(String.valueOf(index), new DataType(PredType.NATIVE_LONG()), new DataSpace(2, dimensions));
      long[] array = data.toArray();
      dataset.write(new LongPointer(array), new DataType(PredType.NATIVE_LONG()));
      dataset.close();
   }

   /**
    * Stores a 2D float array passed as TFloatArrayList into an HDF5 dataset
    *
    * @param group The HDF5 group where the requested is stored
    * @param index The index of the dataset within the requested group.
    * @param data  The float data for the 2D matrix to be stored
    * @param rows  Number of rows in the 2D matrix to be stored
    * @param cols  Number of columns in the 2D matrix to be stored
    */
   public static void storeFloatArray2D(Group group, long index, TFloatArrayList data, int rows, int cols)
   {
      // Log the data buffer as separate dataset with same index
      long[] dimensions = {rows, cols};
      DataSet dataset = group.createDataSet(String.valueOf(index), new DataType(PredType.NATIVE_FLOAT()), new DataSpace(2, dimensions));
      float[] dataObject = data.toArray();
      dataset.write(new FloatPointer(dataObject), new DataType(PredType.NATIVE_FLOAT()));
      dataset.close();
   }

   /**
    * Loads a 2D float array from an HDF5 dataset
    *
    * @param group The HDF5 group where the requested float array is stored
    * @param index The index of the dataset within the requested group.
    */
   public static float[] loadFloatArray(Group group, long index)
   {
      DataSet dataset = group.openDataSet(String.valueOf(index));

      int size = extractShape(dataset, 0);
      int cols = extractShape(dataset, 1);

      LogTools.info("Shape: {} {}", size, cols);

      float[] floatArray = new float[size * cols];

      FloatPointer floatPointer = new FloatPointer(floatArray);
      dataset.read(floatPointer, new DataType(PredType.NATIVE_FLOAT()));
      floatPointer.get(floatArray, 0, floatArray.length);

      dataset.close();

      return floatArray;
   }

   /**
    * Loads a float array from an HDF5 dataset
    *
    * @param group The HDF5 group where the requested float array is stored
    * @param index The index of the dataset within the requested group.
    */
   public static void loadFloatArray(Group group, long index, float[] arrayToPack)
   {
      DataSet dataset = group.openDataSet(String.valueOf(index));

      FloatPointer floatPointer = new FloatPointer(arrayToPack);
      dataset.read(floatPointer, new DataType(PredType.NATIVE_FLOAT()));
      floatPointer.get(arrayToPack, 0, arrayToPack.length);

      dataset.close();
   }

   /**
    * Extracts a list of all topic names stored inside the HDF5 file recursively.
    *
    * @param file The HDF5 file to be explored.
    * @return List of all topic names inside the HDF5 file.
    */
   public static ArrayList<String> getTopicNames(H5File file)
   {
      ArrayList<String> names = new ArrayList<>();

      for (int i = 0; i < file.getNumObjs(); i++)
      {
         String objectName = file.getObjnameByIdx(i).getString();
         Group group = file.openGroup(objectName);
         recursivelyExploreHDF5File(group, names, objectName);
      }

      return names;
   }

   /* Recursive function to go through the various topic names in the HDF5 hierarchically. Called by
    *  getTopicNames() to create a list of topic names stored within the file. */
   private static void recursivelyExploreHDF5File(Group group, ArrayList<String> names, String prefix)
   {
      int numberOfGroups = 0;
      for (int i = 0; i < group.getNumObjs(); i++)
      {
         BytePointer objectBytePointer = group.getObjnameByIdx(i);
         if (group.childObjType(objectBytePointer) == hdf5.H5O_TYPE_GROUP)
         {
            numberOfGroups++;
            String groupName = group.getObjnameByIdx(i).getString();
            Group groupHandle = group.openGroup(groupName);
            recursivelyExploreHDF5File(groupHandle, names, prefix + "/" + groupName);
         }

         // TODO: Handle the case where the type of the object is a dataset.

         //         if (group.childObjType(objPtr) == H5O_TYPE_DATASET) {
         //            String dsName = group.getObjnameByIdx(i).getString();
         //            System.out.println("Dataset: " + dsName + "\t" + "Prefix: " + prefix);
         //         }

         //         System.out.println("ExploreH5: " + group.getObjnameByIdx(i).getString());
      }

      if (numberOfGroups == 0)
      {
         names.add(prefix);
      }
   }

   // TODO: Complete the method to store a double matrix
   public static void storeMatrix(Group group, double[] data)
   {
      long[] dimensions = {5, 5};
      if (group.nameExists(String.valueOf(0)))
      {
         DataSet dataset = group.openDataSet(String.valueOf(0));
         dataset.write(new DoublePointer(data), new DataType(PredType.NATIVE_DOUBLE()));
         dataset.close();
      }
      else
      {
         DataSet dataset = group.createDataSet(String.valueOf(0), new DataType(PredType.NATIVE_DOUBLE()), new DataSpace(2, dimensions));
         dataset.close();
      }
   }

   // TODO: Does not work yet. Needs to be fixed.
   public static void storeRawByteArray(Group group, long index, byte[] data, long size)
   {
      LogTools.info("Store Byte Array: {} {}", index, size);
      long[] dimensions = {size};
      LogTools.info("Creating Dataset: {} {}", group.toString(), String.valueOf(index));

      DataSpace dataSpace = new DataSpace(1, dimensions);
      DataSet dataset = group.createDataSet(String.valueOf(index), new DataType(PredType.NATIVE_UCHAR()), dataSpace);

      dataset.write(new BytePointer(data), new DataType(PredType.NATIVE_UCHAR()));

      dataSpace.close();
      dataset.close();
   }

   /* TODO: Does not work yet. Needs to be fixed. */

   public static byte[] loadRawByteArray(Group group, int index)
   {
      DataSet dataset = group.openDataSet(String.valueOf(index));

      int size = extractShape(dataset, 0);

      byte[] byteArray = new byte[size];

      BytePointer bytePointer = new BytePointer(byteArray);
      dataset.read(bytePointer, new DataType(PredType.NATIVE_UCHAR()));
      bytePointer.get(byteArray, 0, byteArray.length);

      return byteArray;
   }

   // TODO: Complete this method to load timestamps associated with any HDF5 dataset object.
   public static ArrayList<Float> getTimestamps(Group group)
   {
      ArrayList<Float> timestamps = new ArrayList<>();
      return timestamps;
   }
}