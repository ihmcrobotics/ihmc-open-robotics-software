package us.ihmc.perception.logging;

import org.bytedeco.hdf5.*;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.*;
import us.ihmc.log.LogTools;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

public class HDF5Tools
{
   /**
    * Method to generate HDF5 file name for a new log file
    */
   public static String generateFileName()
   {
      return generateFileName("PerceptionLog");
   }

   public static String generateFileName(String suffix)
   {
      SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      String logFileName = dateFormat.format(new Date()) + "_" + suffix + ".hdf5";
      return logFileName;
   }

   /**
    * Extracts the shape of a dataset and outputs the length along the requested dimension/axis (dim)
    *
    * @param dataSet        The HDF5 dataset of which the shape is requested
    * @param dimensionIndex The dimension or axis along which the length is requested
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
    * Stores an int[] array into the requested HDF5 dataset.
    *
    * @param group The HDF5 group where the int[] array data is to be stored
    * @param index The index of the dataset within the requested group.
    * @param data  The int[] array to be stored into the HDF5 file.
    * @param size  Size of the relevant part of the data to be stored
    */
   @Deprecated
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
    * Stores a 2D long array passed as TLongArrayList into an HDF5 dataset
    *
    * @param group   The HDF5 group where the requested is stored
    * @param index   The index of the dataset within the requested group.
    * @param pointer The long data pointer for the 2D matrix to be stored
    * @param rows    Number of rows in the 2D matrix to be stored
    * @param cols    Number of columns in the 2D matrix to be stored
    */
   public void storeLongArray2D(Group group, long index, LongPointer pointer, int rows, int cols)
   {
      // Log the timestamps buffer as separate dataset with same index
      long[] dimensions = {rows, cols};
      DataSet dataset = group.createDataSet(String.valueOf(index), new DataType(PredType.NATIVE_LONG()), new DataSpace(2, dimensions));
      dataset.write(pointer, new DataType(PredType.NATIVE_LONG()));
      dataset._close();
   }

   /**
    * Stores a 2D float array passed as TFloatArrayList into an HDF5 dataset
    *
    * @param group   The HDF5 group where the requested is stored
    * @param index   The index of the dataset within the requested group.
    * @param pointer The float data pointer for the 2D matrix to be stored
    * @param rows    Number of rows in the 2D matrix to be stored
    * @param cols    Number of columns in the 2D matrix to be stored
    */
   public void storeFloatArray2D(Group group, long index, FloatPointer pointer, int rows, int cols)
   {
      LogTools.debug("Store Float Array 2D: Index: {} Rows: {} Cols: {}", index, rows, cols);

      // Log the data buffer as separate dataset with same index
      long[] dimensions = {rows, cols};
      DataSet dataset = group.createDataSet(String.valueOf(index), new DataType(PredType.NATIVE_FLOAT()), new DataSpace(2, dimensions));
      dataset.write(pointer, new DataType(PredType.NATIVE_FLOAT()));
      dataset._close();
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

   /**
    * Stores bytes from a BytePointer into an HDF5 dataset
    *
    * @param group          The HDF5 group where the requested is stored
    * @param index          The index of the dataset within the requested group.
    * @param srcBytePointer The BytePointer containing the bytes to be stored
    */
   public void storeBytes(Group group, long index, BytePointer srcBytePointer)
   {
      long size = srcBytePointer.limit();

      LogTools.debug("Store Byte Array: Index: {} Size: {}", index, size);
      long[] dims = {size};

      DataType dataType = new DataType(PredType.NATIVE_UINT8());
      DataSpace dataSpace = new DataSpace(1, dims);
      DataSet dataSet = group.createDataSet(String.valueOf(index), dataType, dataSpace);

      dataSet.write((Pointer) srcBytePointer, dataType);
   }

   /**
    * Loads bytes from an HDF5 dataset into a BytePointer
    *
    * @param group       The HDF5 group where the requested is stored
    * @param index       The index of the dataset within the requested group.
    * @param bytePointer The BytePointer to be filled with the bytes from the HDF5 dataset
    * @return The number of bytes loaded into the BytePointer
    */
   public int loadBytes(Group group, int index, BytePointer bytePointer)
   {
      DataSet dataSet = group.openDataSet(String.valueOf(index));

      long size = dataSet.getInMemDataSize();
      if (size > bytePointer.capacity())
      {
         LogTools.warn("Byte array is too small to hold the data. Resizing to {} from {}", 2 * size, bytePointer.capacity());
         bytePointer.capacity(2 * size);
      }

      long dtype = DataType.getHDFObjType(dataSet.getId());
      LogTools.debug("Index: {} Size: {}", index, size);

      DataType dataType = new DataType(PredType.NATIVE_UINT8());
      dataSet.read((Pointer) bytePointer, dataType);

      return (int) size;
   }

   /**
    * Writes an int attribute to the HDF5 group.
    *
    * @param name
    * @param value
    */
   public void writeIntAttribute(Group group, String name, int value)
   {
      long[] dims = {1};
      DataType dataType = new DataType(PredType.NATIVE_INT32());
      DataSpace dataSpace = new DataSpace(1, dims);
      Attribute attribute = group.createAttribute(name, dataType, dataSpace);
      IntPointer pointer = new IntPointer(1);
      pointer.put(0, value);
      attribute.write(dataType, pointer);
      attribute._close();
   }

   /**
    * Writes a long attribute to the HDF5 group.
    *
    * @param group HDF5 group to write the attribute to.
    * @param name  Attribute name.
    * @param value Attribute value.
    */
   public void writeLongAttribute(Group group, String name, long value)
   {
      long[] dims = {1};
      DataType dataType = new DataType(PredType.NATIVE_INT64());
      DataSpace dataSpace = new DataSpace(1, dims);
      Attribute attribute = group.createAttribute(name, dataType, dataSpace);
      LongPointer pointer = new LongPointer(1);
      pointer.put(0, value);
      attribute.write(dataType, pointer);
      attribute._close();
   }

   /**
    * Writes a float attribute to the HDF5 group.
    *
    * @param group HDF5 group to write the attribute to.
    * @param name  Attribute name.
    * @param value Attribute value.
    */
   public void writeFloatAttribute(Group group, String name, float value)
   {
      long[] dims = {1};
      DataType dataType = new DataType(PredType.NATIVE_FLOAT());
      DataSpace dataSpace = new DataSpace(1, dims);
      Attribute attribute = group.createAttribute(name, dataType, dataSpace);
      FloatPointer pointer = new FloatPointer(1);
      pointer.put(0, value);
      attribute.write(dataType, pointer);
      attribute._close();
   }

   /**
    * Writes a double attribute to the HDF5 group.
    *
    * @param group HDF5 group to write the attribute to.
    * @param name  Attribute name.
    * @param value Attribute value.
    */
   public void writeDoubleAttribute(Group group, String name, double value)
   {
      long[] dims = {1};
      DataType dataType = new DataType(PredType.NATIVE_DOUBLE());
      DataSpace dataSpace = new DataSpace(1, dims);
      Attribute attribute = group.createAttribute(name, dataType, dataSpace);
      DoublePointer pointer = new DoublePointer(1);
      pointer.put(0, value);
      attribute.write(dataType, pointer);
      attribute._close();
   }

   /**
    * Writes a string attribute to the HDF5 group.
    *
    * @param group HDF5 group to write the attribute to.
    * @param name  Attribute name.
    * @param value Attribute value.
    */
   public void writeStringAttribute(Group group, String name, String value)
   {
      long[] dims = {value.length() + 1};
      DataType dataType = new DataType(PredType.C_S1());
      DataSpace dataSpace = new DataSpace(1, dims);
      Attribute attribute = group.createAttribute(name, dataType, dataSpace);
      attribute.write(dataType, value);
      attribute._close();
   }

   /**
    * Reads an int attribute to the HDF5 group.
    *
    * @param group HDF5 group to read the attribute from.
    * @param name  Attribute name.
    */
   public int readIntAttribute(Group group, String name)
   {
      Attribute sizeAttribute = group.openAttribute(name);
      IntPointer sizeAttributePointer = new IntPointer(1);
      DataType dataType = new DataType(PredType.NATIVE_INT32());
      sizeAttribute.read(dataType, sizeAttributePointer);
      sizeAttribute._close();
      return sizeAttributePointer.get(0);
   }

   /**
    * Reads a long attribute to the HDF5 group.
    *
    * @param group HDF5 group to read the attribute from.
    * @param name  Attribute name.
    */
   public long readLongAttribute(Group group, String name)
   {
      Attribute sizeAttribute = group.openAttribute(name);
      LongPointer sizeAttributePointer = new LongPointer(1);
      DataType dataType = new DataType(PredType.NATIVE_INT64());
      sizeAttribute.read(dataType, sizeAttributePointer);
      sizeAttribute._close();
      return sizeAttributePointer.get(0);
   }

   /**
    * Reads a float attribute to the HDF5 group.
    *
    * @param group HDF5 group to read the attribute from.
    * @param name  Attribute name.
    */
   public float readFloatAttribute(Group group, String name)
   {
      Attribute sizeAttribute = group.openAttribute(name);
      FloatPointer sizeAttributePointer = new FloatPointer(1);
      DataType dataType = new DataType(PredType.NATIVE_FLOAT());
      sizeAttribute.read(dataType, sizeAttributePointer);
      sizeAttribute._close();
      return sizeAttributePointer.get(0);
   }

   /**
    * Reads a double attribute to the HDF5 group.
    *
    * @param group HDF5 group to read the attribute from.
    * @param name  Attribute name.
    */
   public double readDoubleAttribute(Group group, String name)
   {
      Attribute sizeAttribute = group.openAttribute(name);
      DoublePointer sizeAttributePointer = new DoublePointer(1);
      DataType dataType = new DataType(PredType.NATIVE_DOUBLE());
      sizeAttribute.read(dataType, sizeAttributePointer);
      sizeAttribute._close();
      return sizeAttributePointer.get(0);
   }

   /**
    * Reads a string attribute to the HDF5 group.
    *
    * @param group HDF5 group to read the attribute from.
    * @param name  Attribute name.
    */
   public String readStringAttribute(Group group, String name)
   {
      Attribute attribute = group.openAttribute(name);
      DataType dataType = new DataType(PredType.C_S1());
      DataSpace dataSpace = attribute.getSpace();
      long[] dims = new long[2];
      dataSpace.getSimpleExtentDims(dims);

      LogTools.info("Reading attribute: " + name + " with size: " + dims[0]);

      BytePointer charPointer = new BytePointer(dims[0] + 1);
      attribute.read(dataType, charPointer);
      attribute._close();
      return new String(charPointer.getStringBytes(), StandardCharsets.UTF_8);
   }

   /**
    * Loads a byte[] array from an HDF5 dataset within the requested group. Uses loadIntArray() as the backing method.
    *
    * @param group The HDF5 group where the requested pointcloud is stored
    * @param index The index of the dataset within the requested group.
    * @return The byte[] array with the data from HDF5 dataset.
    */
   @Deprecated
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
   @Deprecated
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
    * Loads a 2D float array from an HDF5 dataset
    *
    * @param group The HDF5 group where the requested float array is stored
    * @param index The index of the dataset within the requested group.
    */
   @Deprecated
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
    * Stores a byte[] array into the requested HDF5 dataset. Uses storeIntArray() as the backing method.
    *
    * @param group The HDF5 group where the byte[] array data is to be stored
    * @param index The index of the dataset within the requested group.
    * @param data  The byte[] array to be stored into the HDF5 file.
    * @param size  Size of the relevant part of the data to be stored
    */
   @Deprecated
   public void storeByteArray(Group group, long index, byte[] data, int size)
   {
      ByteBuffer buffer = ByteBuffer.wrap(data, 0, size);
      IntBuffer intBuffer = buffer.asIntBuffer();

      int intCount = (size / Integer.BYTES) + 1;
      int[] array = new int[intCount];
      intBuffer.get(array, 0, intCount - 1);

      storeIntArray(group, index, array, intCount);
   }
}