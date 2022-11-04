package us.ihmc.rdx.logging;

import net.jpountz.lz4.LZ4Factory;
import net.jpountz.lz4.LZ4FastDecompressor;
import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;
import us.ihmc.perception.*;
import us.ihmc.perception.logging.HDF5Manager;
import us.ihmc.perception.logging.HDF5Tools;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;

public class PerceptionDataLoader
{
   private HDF5Manager hdf5Manager;

   private final LZ4FastDecompressor lz4Decompressor = LZ4Factory.nativeInstance().fastDecompressor();

   private int latestSegmentIndex = -1;

   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel unpackPointCloudKernel;
   private OpenCLFloatBuffer pointCloudVertexBuffer;
   private OpenCLIntBuffer decompressedOpenCLIntBuffer;
   private OpenCLFloatBuffer parametersOpenCLFloatBuffer;

   private int index = 0;

   private String filePath;

   private int os0128Multiplier = 2;
   private int pointsPerSegment = 131072 * os0128Multiplier;
   private int numberOfSegments = 1;

   private FloatBuffer hostPointCloudBuffer;

   public PerceptionDataLoader(String filePath)
   {
      this.filePath = filePath;
      hdf5Manager = new HDF5Manager(filePath, hdf5.H5F_ACC_RDONLY);

      openCLManager = new OpenCLManager();
      openCLManager.create();
      openCLProgram = openCLManager.loadProgram("FusedSensorPointCloudSubscriberVisualizer");
      unpackPointCloudKernel = openCLManager.createKernel(openCLProgram, "unpackPointCloud");

      hostPointCloudBuffer = FloatBuffer.allocate(pointsPerSegment * 8);

      parametersOpenCLFloatBuffer = new OpenCLFloatBuffer(2);
      parametersOpenCLFloatBuffer.createOpenCLBufferObject(openCLManager);
      decompressedOpenCLIntBuffer = new OpenCLIntBuffer(pointsPerSegment * 4);
      decompressedOpenCLIntBuffer.createOpenCLBufferObject(openCLManager);
      pointCloudVertexBuffer = new OpenCLFloatBuffer(pointsPerSegment * 8, hostPointCloudBuffer);
      pointCloudVertexBuffer.createOpenCLBufferObject(openCLManager);
   }

   public void loadPointCloud(String namespace, int index, RecyclingArrayList<Point3D32> points)
   {
      HDF5Tools.loadPointCloud(hdf5Manager.getGroup(namespace), index, points);
   }

   public FloatBuffer loadCompressedPointCloud(String namespace, int index)
   {
      Group group = hdf5Manager.getGroup(namespace);
      byte[] compressedByteArray = HDF5Tools.loadByteArray(group, index);

      LogTools.info("Byte Array: {} {} {}", compressedByteArray[0], compressedByteArray[1], compressedByteArray[2]);

      ByteBuffer compressedByteBuffer = ByteBuffer.wrap(compressedByteArray);

      int numberOfBytes = compressedByteArray.length;

      lz4Decompressor.decompress(compressedByteBuffer, decompressedOpenCLIntBuffer.getBackingDirectByteBuffer());
      decompressedOpenCLIntBuffer.getBackingDirectByteBuffer().rewind();

//      latestSegmentIndex = (int) fusedMessage.getSegmentIndex();

      parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(0, 0);
      parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(1, 0.5f);
      parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(2, pointsPerSegment);

      parametersOpenCLFloatBuffer.writeOpenCLBufferObject(openCLManager);
      decompressedOpenCLIntBuffer.writeOpenCLBufferObject(openCLManager);

      openCLManager.setKernelArgument(unpackPointCloudKernel, 0, parametersOpenCLFloatBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 1, decompressedOpenCLIntBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 2, pointCloudVertexBuffer.getOpenCLBufferObject());
      openCLManager.execute1D(unpackPointCloudKernel, pointsPerSegment);
      pointCloudVertexBuffer.readOpenCLBufferObject(openCLManager);

      return hostPointCloudBuffer;
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
