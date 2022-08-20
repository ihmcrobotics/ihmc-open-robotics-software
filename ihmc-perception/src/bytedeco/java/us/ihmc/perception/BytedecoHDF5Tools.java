package us.ihmc.perception;

import java.util.ArrayList;

import org.bytedeco.javacpp.*;
import org.bytedeco.hdf5.*;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;

import static org.bytedeco.hdf5.global.hdf5.*;

public class BytedecoHDF5Tools
{

   static final String FILE_NAME = "/home/quantum/Workspace/Storage/Other/Temp/ISR_Logs/atlas_01.h5";
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

   public static ArrayList<Point3D32> loadPointCloud(String filename)
   {


      ArrayList<Point3D32> points = new ArrayList<>();

      H5File file = new H5File(FILE_NAME, H5F_ACC_RDONLY);
      DataSet dataset = file.openDataSet("/os_cloud_node/points/0");
      float[] pointsBuffer = new float[DIM0 * DIM1 * 3];

      DataSpace space = dataset.getSpace();
      int nbDims = space.getSimpleExtentNdims();

      System.out.println("Shape:" + extractShape(dataset, 0) + " " + extractShape(dataset, 1));

      FloatPointer p = new FloatPointer(pointsBuffer);

      dataset.read(p, PredType.NATIVE_FLOAT());
      p.get(pointsBuffer);

      for(int i = 0; i<pointsBuffer.length; i+=3)
      {
         Point3D32 point = new Point3D32(pointsBuffer[i], pointsBuffer[i+1], pointsBuffer[i+2]);
         points.add(point);
      }

      file.close();

      return points;
   }

   public static void main(String[] args)
   {
      long[] dims = { DIM0, DIM1 };        // dataset dimensions
      long[] chunk_dims = { 20, 20 };        // chunk dimensions
      int[] buf = new int[DIM0 * DIM1];


      try {
         org.bytedeco.hdf5.Exception.dontPrint();

         ArrayList<Point3D32> points = BytedecoHDF5Tools.loadPointCloud(FILE_NAME);

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
