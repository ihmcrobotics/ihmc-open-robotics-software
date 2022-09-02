package us.ihmc.perception;

import java.util.ArrayList;

import org.bytedeco.javacpp.*;
import org.bytedeco.hdf5.*;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;

import static org.bytedeco.hdf5.global.hdf5.*;

public class BytedecoHDF5Tools
{

   static final String FILE_NAME = "/home/bmishra/Workspace/Data/Atlas_Logs/ROSBags/atlas_perception_run_1.h5";
   static final String DATASET_NAME = "/os_cloud_node/points/0";
   static final int DIM0 = 2048;
   static final int DIM1 = 64;

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

   public static void loadPointCloud(String filename, RecyclingArrayList<Point3D32> points)
   {
      H5File file = new H5File(filename, H5F_ACC_RDONLY);
      DataSet dataset = file.openDataSet("/os_cloud_node/points/0");
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

   public static void main(String[] args)
   {
      long[] dims = { DIM0, DIM1 };        // dataset dimensions
      long[] chunk_dims = { 20, 20 };        // chunk dimensions
      int[] buf = new int[DIM0 * DIM1];


      try {
         org.bytedeco.hdf5.Exception.dontPrint();

         RecyclingArrayList<Point3D32> points = new RecyclingArrayList<>(100000, Point3D32::new);

         BytedecoHDF5Tools.loadPointCloud(FILE_NAME, points);

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
