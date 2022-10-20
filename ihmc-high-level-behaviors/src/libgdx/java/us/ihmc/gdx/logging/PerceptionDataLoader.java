package us.ihmc.gdx.logging;

import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.HDF5Manager;
import us.ihmc.perception.HDF5Tools;

public class PerceptionDataLoader
{
   private HDF5Manager hdf5Manager;

   private int index = 0;

   private String filePath;

   public PerceptionDataLoader(String filePath)
   {
      this.filePath = filePath;
      hdf5Manager = new HDF5Manager(filePath, hdf5.H5F_ACC_RDONLY);
   }

   public void loadPointCloud(String namespace, int index, RecyclingArrayList<Point3D32> points)
   {
      HDF5Tools.loadPointCloud(hdf5Manager.getGroup(namespace), index, points);
   }

   public void loadCompressedPointCloud(String namespace, int index, RecyclingArrayList<Point3D32> points)
   {
      HDF5Tools.loadPointCloud(hdf5Manager.getGroup(namespace), index, points);
   }

   public void loadImage(String namespace, int index, Mat mat)
   {
      HDF5Tools.loadImage(hdf5Manager.getGroup(namespace), index, mat);
   }

   public String getFilePath() {
      return filePath;
   }

   public HDF5Manager getHDF5Manager() {
      return hdf5Manager;
   }

   public static void main(String[] args)
   {
      String FILE_NAME = "/home/quantum/Workspace/Data/Atlas_Logs/ROSBags/atlas_perception_run_1.hdf5";
      PerceptionDataLoader loader = new PerceptionDataLoader(FILE_NAME);
      //      ScheduledExecutorService executorService = ExecutorServiceTools.newSingleThreadScheduledExecutor(loader.getClass(), ExecutorServiceTools.ExceptionHandling.CANCEL_AND_REPORT);
      //      executorService.scheduleAtFixedRate(loader::loadNextDataFrame, 0, 100, TimeUnit.MILLISECONDS);

      RecyclingArrayList<Point3D32> points = new RecyclingArrayList<>(200000, Point3D32::new);
      for (int i = 0; i < loader.getHDF5Manager().getCount("/os_cloud_node/points"); i++)
      {
         loader.loadPointCloud("os_cloud_node/points", i, points);
      }
   }
}
