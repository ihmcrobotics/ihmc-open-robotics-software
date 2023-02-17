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
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;

public class HDF5Tools
{
   public static final String HDF5_FILE_EXTENSION = ".hdf5";
   static final int NUMBER_OF_FIELDS_PER_POINT = 3;

   /**
    * Extracts the shape of a dataset and outputs the length along the requested dimension/axis (dim)
    *
    * @param dataSet The HDF5 dataset of which the shape is requested
    * @param dimensionIndex     The dimension or axis along which the length is requested
    * @return The length of the dataset along the requested dimension or axis
    */
   private int extractShape(DataSet dataSet, int dimensionIndex)
   {
      DataSpace dataSpace = dataSet.getSpace();
      int numberOfDimensions = dataSpace.getSimpleExtentNdims();
      long[] shape = new long[numberOfDimensions];
      dataSpace.getSimpleExtentDims(shape);
      if (dimensionIndex < numberOfDimensions)
      {
         return (int) shape[dimensionIndex];
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
   public void loadPointCloud(Group group, int index, RecyclingArrayList<Point3D32> pointListToPack, int rows, int cols)
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
   public byte[] loadByteArray(Group group, int index)
   {
      int[] loadedIntArray = loadIntArray(group, index);

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
   public int[] loadIntArray(Group group, int index)
   {
      DataSet dataset = group.openDataSet(String.valueOf(index));

      int size = extractShape(dataset, 0);
      int[] intArray = new int[size];

      IntPointer intPointer = new IntPointer(intArray);
      dataset.read(intPointer, new DataType(PredType.NATIVE_INT()));
      intPointer.get(intArray, 0, intArray.length);

      dataset._close();

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
   public void storeByteArray(Group group, long index, byte[] data, int size)
   {
      ByteBuffer buffer = ByteBuffer.wrap(data, 0, size);
      IntBuffer intBuffer = buffer.asIntBuffer();

      int intCount = (size / Integer.BYTES) + 1;
      int[] array = new int[intCount];
      intBuffer.get(array, 0, intCount - 1);

      storeIntArray(group, index, array, intCount);
   }

   /**
    * Stores an int[] array into the requested HDF5 dataset.
    *
    * @param group The HDF5 group where the int[] array data is to be stored
    * @param index The index of the dataset within the requested group.
    * @param data  The int[] array to be stored into the HDF5 file.
    * @param size  Size of the relevant part of the data to be stored
    */
   public void storeIntArray(Group group, long index, int[] data, long size)
   {
      LogTools.info("Store Int Array: Index: {} Size: {}", index, size);
      long[] dimensions = {size};

      DataSpace dataSpace = new DataSpace(1, dimensions);
      DataSet dataset = group.createDataSet(String.valueOf(index), new DataType(PredType.NATIVE_INT()), dataSpace);

      dataset.write(new IntPointer(data), new DataType(PredType.NATIVE_INT()));

      dataSpace._close();
      dataset._close();
   }

   /**
    * Stores a pointcloud stored in an ArrayList of 3D points into an HDF5 dataset
    *
    * @param group           The HDF5 group where the requested pointcloud is stored
    * @param index           The index of the dataset within the requested group.
    * @param pointListToPack The final packed 3D point list with pointcloud
    */
   public void storePointCloud(Group group, long index, ArrayList<Point3D> pointListToPack)
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
   public void storeLongArray2D(Group group, long index, TLongArrayList data, int rows, int cols)
   {
      // Log the timestamps buffer as separate dataset with same index
      long[] dimensions = {rows, cols};
      DataSet dataset = group.createDataSet(String.valueOf(index), new DataType(PredType.NATIVE_LONG()), new DataSpace(2, dimensions));
      long[] array = data.toArray();
      dataset.write(new LongPointer(array), new DataType(PredType.NATIVE_LONG()));
      dataset._close();
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
   public void storeFloatArray2D(Group group, long index, TFloatArrayList data, int rows, int cols)
   {
      // Log the data buffer as separate dataset with same index
      long[] dimensions = {rows, cols};
      DataSet dataset = group.createDataSet(String.valueOf(index), new DataType(PredType.NATIVE_FLOAT()), new DataSpace(2, dimensions));
      float[] dataObject = data.toArray();
      dataset.write(new FloatPointer(dataObject), new DataType(PredType.NATIVE_FLOAT()));
      dataset._close();
   }

   /**
    * Loads a 2D float array from an HDF5 dataset
    *
    * @param group The HDF5 group where the requested float array is stored
    * @param index The index of the dataset within the requested group.
    */
   public float[] loadFloatArray(Group group, long index)
   {
      DataSet dataset = group.openDataSet(String.valueOf(index));

      int size = extractShape(dataset, 0);
      int cols = extractShape(dataset, 1);

      float[] floatArray = new float[size * cols];

      FloatPointer floatPointer = new FloatPointer(floatArray);
      dataset.read(floatPointer, new DataType(PredType.NATIVE_FLOAT()));
      floatPointer.get(floatArray, 0, floatArray.length);

      dataset._close();

      return floatArray;
   }

   /**
    * Loads a float array from an HDF5 dataset
    *
    * @param group The HDF5 group where the requested float array is stored
    * @param index The index of the dataset within the requested group.
    */
   public void loadFloatArray(Group group, long index, float[] arrayToPack)
   {
      DataSet dataset = group.openDataSet(String.valueOf(index));

      FloatPointer floatPointer = new FloatPointer(arrayToPack);
      dataset.read(floatPointer, new DataType(PredType.NATIVE_FLOAT()));
      floatPointer.get(arrayToPack, 0, arrayToPack.length);

      dataset._close();
   }

   /**
    * Extracts a list of all topic names stored inside the HDF5 file recursively.
    *
    * @param file The HDF5 file to be explored.
    * @return List of all topic names inside the HDF5 file.
    */
   public ArrayList<String> findTopicNames(H5File file)
   {
      ArrayList<String> names = new ArrayList<>();

      for (int i = 0; i < file.getNumObjs(); i++)
      {
         BytePointer objnameByIdx = file.getObjnameByIdx(i);
         if (file.childObjType(objnameByIdx) == hdf5.H5O_TYPE_GROUP)
         {
            String partialTopicName = "/" + objnameByIdx.getString();
            Group group = file.openGroup(objnameByIdx);
            recursivelyFindTopicNames(group, names, partialTopicName);
            group._close();
         }
      }

      return names;
   }

   /**
    * Create a new handle for the group requested in the namespace
    *
    * @param namespace The namespace for which a group handle has been requested
    * @return The HDF5 group handle for the requested namespace
    */
   public Group createGroup(H5File file, String namespace)
   {
      Path path = Paths.get(namespace);
      Group group = null;

      for (int i = 0; i < path.getNameCount(); i++)
      {
         String name = path.subpath(0, i + 1).toString();
         if (!file.nameExists(name))
         {
            if(group != null)
            {
               group._close();
               group = null;
            }
            group = file.createGroup(name);
         }
      }

      if (group == null)
      {
         group = file.openGroup(namespace);
      }

      return group;
   }

   /* Recursive function to go through the various topic names in the HDF5 hierarchically. Called by
    *  getTopicNames() to create a list of topic names stored within the file. */
   private int recursivelyFindTopicNames(Group group, ArrayList<String> names, String topicNameInProgress)
   {
      int numberOfGroups = 0;
      for (int i = 0; i < group.getNumObjs(); i++)
      {
         BytePointer objnameByIdx = group.getObjnameByIdx(i);
         if (group.childObjType(objnameByIdx) == hdf5.H5O_TYPE_GROUP)
         {
            String topicNamePart = objnameByIdx.getString();
            Group groupHandle = group.openGroup(objnameByIdx);
            numberOfGroups += recursivelyFindTopicNames(groupHandle, names, topicNameInProgress + "/" + topicNamePart);
            groupHandle._close();
         }

         // TODO: Handle the case where the type of the object is a dataset.

         //         if (group.childObjType(objPtr) == H5O_TYPE_DATASET) {
         //            String dsName = group.getObjnameByIdx(i).getString();
         //            System.out.println("Dataset: " + dsName + "\t" + "Prefix: " + topicNameInProgress);
         //         }

         //         System.out.println("ExploreH5: " + group.getObjnameByIdx(i).getString());
      }

      // If leaf node, that's a topic, so let's add it to the list
      if (numberOfGroups == 0)
      {
         names.add(topicNameInProgress + "/");
      }

      return numberOfGroups;
   }

   // TODO: Complete the method to store a double matrix
   public void storeMatrix(Group group, double[] data)
   {
      long[] dimensions = {5, 5};
      if (group.nameExists(String.valueOf(0)))
      {
         DataSet dataset = group.openDataSet(String.valueOf(0));
         dataset.write(new DoublePointer(data), new DataType(PredType.NATIVE_DOUBLE()));
         dataset._close();
      }
      else
      {
         DataSet dataset = group.createDataSet(String.valueOf(0), new DataType(PredType.NATIVE_DOUBLE()), new DataSpace(2, dimensions));
         dataset._close();
      }
   }

   // TODO: Does not work yet. Needs to be fixed.
   public void storeRawByteArray(Group group, long index, byte[] data, long size)
   {
      LogTools.info("Store Byte Array: {} {}", index, size);
      long[] dimensions = {size};
      LogTools.info("Creating Dataset: {} {}", group.toString(), String.valueOf(index));

      DataSpace dataSpace = new DataSpace(1, dimensions);
      DataSet dataset = group.createDataSet(String.valueOf(index), new DataType(PredType.NATIVE_UCHAR()), dataSpace);

      dataset.write(new BytePointer(data), new DataType(PredType.NATIVE_UCHAR()));

      dataSpace._close();
      dataset._close();
   }

   public void storeBytes(Group group, long index, BytePointer srcBytePointer)
   {
      long size = srcBytePointer.limit();

      LogTools.info("Store Byte Array: Index: {} Size: {}", index, size);
      long[] dims = {size};

      DataType dataType = new DataType(PredType.NATIVE_UINT8());
      DataSpace dataSpace = new DataSpace(1, dims);
      DataSet dataSet = group.createDataSet(String.valueOf(index), dataType, dataSpace);

      dataSet.write((Pointer) srcBytePointer, dataType);
   }

   /* TODO: Does not work yet. Needs to be fixed. */

   public int loadBytes(Group group, int index, BytePointer bytePointer)
   {
      DataSet dataSet = group.openDataSet(String.valueOf(index));

      long size = dataSet.getInMemDataSize();
      if(size > bytePointer.capacity())
      {
         LogTools.warn("Byte array is too small to hold the data. Resizing to {} from {}", 2 * size, bytePointer.capacity());
         bytePointer.capacity(2 * size);
      }

      long dtype = DataType.getHDFObjType(dataSet.getId());
      LogTools.info("Dataset DType: {}", dtype);

      DataType dataType = new DataType(PredType.NATIVE_UINT8());
      dataSet.read((Pointer) bytePointer, dataType);

      //dataType._close();
      //dataSet._close();

      return (int) size;
   }

   // TODO: Complete this method to load timestamps associated with any HDF5 dataset object.
   public ArrayList<Float> getTimestamps(Group group)
   {
      ArrayList<Float> timestamps = new ArrayList<>();
      return timestamps;
   }
}