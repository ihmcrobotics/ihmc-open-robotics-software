package us.ihmc.gdx.logging;

import org.bytedeco.hdf5.H5File;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.HDF5Tools;

import static org.bytedeco.hdf5.global.hdf5.H5F_ACC_RDONLY;

public class PerceptionDataLoader
{
   private H5File h5File;

   private int index = 0;

   public PerceptionDataLoader(String filePath)
   {
      h5File = new H5File(filePath, H5F_ACC_RDONLY);
   }

   public void loadPointCloud(String namespace, int index, RecyclingArrayList<Point3D32> points)
   {
      HDF5Tools.loadPointCloud(h5File, points, index);
   }

   public void loadImageRGB8(String namespace, int index, Mat mat)
   {
      HDF5Tools.loadDepthMap(h5File, index, mat);
   }

   public void loadImageR16(String namespace, int index, Mat mat)
   {
   }

   public int getCount(String namespace)
   {
      return HDF5Tools.getCount(h5File, namespace);
   }

   public static void main(String[] args)
   {
      String FILE_NAME = "/home/quantum/Workspace/Data/Atlas_Logs/ROSBags/atlas_perception_run_1.h5";
      PerceptionDataLoader loader = new PerceptionDataLoader(FILE_NAME);
      //      ScheduledExecutorService executorService = ExecutorServiceTools.newSingleThreadScheduledExecutor(loader.getClass(), ExecutorServiceTools.ExceptionHandling.CANCEL_AND_REPORT);
      //      executorService.scheduleAtFixedRate(loader::loadNextDataFrame, 0, 100, TimeUnit.MILLISECONDS);

      RecyclingArrayList<Point3D32> points = new RecyclingArrayList<>(200000, Point3D32::new);
      for (int i = 0; i < loader.getCount("/os_cloud_node/points"); i++)
      {
         loader.loadPointCloud("os_cloud_node/points", i, points);
      }
   }
}
