package us.ihmc.perception;

import org.bytedeco.javacpp.*;
import org.bytedeco.hdf5.*;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;

import static org.bytedeco.hdf5.global.hdf5.*;

public class BytedecoHDF5Tools
{
   static final String FILE_NAME = "/home/bmishra/Workspace/Data/Atlas_Logs/ROSBags/atlas_perception_run_1.h5";
   static final String DATASET_NAME = "/os_cloud_node/points/0";

   static final int PCD_POINT_SIZE = 3;
   static final int DIM0 = 2048;
   static final int DIM1 = 64;

   static final int IMG_WIDTH = 1024;
   static final int IMG_HEIGHT = 768;

   private static int extractShape(DataSet dataSet, int dim) {
      DataSpace space = dataSet.getSpace();
      int nbDims = space.getSimpleExtentNdims();
      long[] shape = new long[nbDims];
      space.getSimpleExtentDims(shape);
      if(dim < nbDims)
      {
         return (int) shape[dim];
      }
      else return 0;
   }

   public static void loadPointCloud(H5File file, RecyclingArrayList<Point3D32> points, int index)
   {
      DataSet dataset = file.openDataSet("/os_cloud_node/points/" + index);
      float[] pointsBuffer = new float[DIM0 * DIM1 * 3];

      DataSpace space = dataset.getSpace();
      int nbDims = space.getSimpleExtentNdims();

      FloatPointer p = new FloatPointer(pointsBuffer);

      dataset.read(p, PredType.NATIVE_FLOAT());
      p.get(pointsBuffer);

      points.clear();
      for(int i = 0; i<pointsBuffer.length; i+=3)
      {
         Point3D32 point = points.add();
         point.set(pointsBuffer[i], pointsBuffer[i+1], pointsBuffer[i+2]);
      }

      file.close();
   }

   public static Mat loadDepthMap(H5File file, int index)
   {
      DataSet dataset = file.openDataSet("/chest_l515/depth/image_rect_raw/" + index);
      byte[] pointsBuffer = new byte[IMG_HEIGHT * IMG_WIDTH * 2];

      DataSpace space = dataset.getSpace();
      int nbDims = space.getSimpleExtentNdims();

      BytePointer p = new BytePointer(pointsBuffer);

      dataset.read(p, PredType.NATIVE_UINT8());
      p.get(pointsBuffer);

      Mat depthU16C1Image = new Mat(768, 1024, opencv_core.CV_16UC1, p);

      file.close();

      return depthU16C1Image;
   }

   public void storePointCloud(H5File file, String namespace, RecyclingArrayList<Point3D32> points)
   {
      DataSet dataset = file.openDataSet(namespace);
      float[] buf = new float[points.size() * PCD_POINT_SIZE];
      for (int i = 0; i <  points.size(); i++)
         for (int j = 0; j < PCD_POINT_SIZE; j++)
            buf[i * PCD_POINT_SIZE + j] = i + j;

      dataset.write(new FloatPointer(buf), new DataType(PredType.NATIVE_FLOAT()));
   }

   public void storeDepthMap(H5File file, String namespace, Mat depthMap)
   {
      DataSet dataset = file.openDataSet(namespace);
      dataset.write(depthMap.data(), PredType.NATIVE_UINT16());
   }

   public static void main(String[] args)
   {
      long[] dims = { DIM0, DIM1 };        // dataset dimensions
      long[] chunk_dims = { 20, 20 };        // chunk dimensions
      int[] buf = new int[DIM0 * DIM1];


      try {
         org.bytedeco.hdf5.Exception.dontPrint();

         RecyclingArrayList<Point3D32> points = new RecyclingArrayList<>(100000, Point3D32::new);

         H5File file = new H5File(FILE_NAME, H5F_ACC_RDONLY);
         BytedecoHDF5Tools.loadPointCloud(file, points, 0);

         for(Point3D32 point : points)
         {
            System.out.println("Point:" + point.getX() + "," + point.getY() + "," + point.getZ());
         }

      }  // end of try block

      // catch failure caused by the H5File, DataSet, and DataSpace operations
      catch (RuntimeException error) {
         System.err.println(error);
         error.printStackTrace();
         System.exit(-1);
      }
   }

}
