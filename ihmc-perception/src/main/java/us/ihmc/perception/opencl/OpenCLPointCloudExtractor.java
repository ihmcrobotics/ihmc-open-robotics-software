package us.ihmc.perception.opencl;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.RawImage;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;

/**
 * Class used to get a RecyclingArrayList of Point3D32s (point cloud) from a 16UC1 depth image
 */
public class OpenCLPointCloudExtractor
{
   private static final int FLOATS_PER_PIXEL = 4; // [0] = depth value, [1] = x, [2] = y, [3] = z (coordinates in world)

   private final OpenCLManager openCLManager = new OpenCLManager();
   private final _cl_program openCLProgram = openCLManager.loadProgram("DepthImageToPointCloudConverter", "PerceptionCommon.cl");
   private final _cl_kernel kernel = openCLManager.createKernel(openCLProgram, "convertDepthImageToPointCloud");
   private final OpenCLFloatParameters parametersBuffer = new OpenCLFloatParameters();
   private final OpenCLRigidBodyTransformParameter depthToWorldTransformParameter = new OpenCLRigidBodyTransformParameter();
   private OpenCLFloatBuffer pointCloudVertexOutput;

   public List<Point3D32> extractPointCloud(RawImage depthImage16UC1)
   {
      RawImage depthImage = depthImage16UC1.get();
      if (depthImage == null || depthImage.getCpuImageMat().isNull() || depthImage.getCpuImageMat().empty())
         return new ArrayList<>();

      int numberOfPixels = depthImage.getImageWidth() * depthImage.getImageHeight();
      if (pointCloudVertexOutput == null)
      {
         pointCloudVertexOutput = new OpenCLFloatBuffer(numberOfPixels * FLOATS_PER_PIXEL);
         pointCloudVertexOutput.createOpenCLBufferObject(openCLManager);
      }

      RigidBodyTransform depthToWorldTransform = new RigidBodyTransform(depthImage.getOrientation(), depthImage.getPosition());
      depthToWorldTransformParameter.setParameter(depthToWorldTransform);
      depthToWorldTransformParameter.writeOpenCLBufferObject(openCLManager);

      parametersBuffer.setParameter(depthImage.getImageWidth());
      parametersBuffer.setParameter(depthImage.getFocalLengthX());
      parametersBuffer.setParameter(depthImage.getFocalLengthY());
      parametersBuffer.setParameter(depthImage.getPrincipalPointX());
      parametersBuffer.setParameter(depthImage.getPrincipalPointY());
      parametersBuffer.setParameter(depthImage.getDepthDiscretization());
      parametersBuffer.writeOpenCLBufferObject(openCLManager);

      BytedecoImage bytedecoDepthImage = new BytedecoImage(depthImage.getCpuImageMat());
      bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
      bytedecoDepthImage.writeOpenCLImage(openCLManager);

      openCLManager.setKernelArgument(kernel, 0, bytedecoDepthImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 1, parametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(kernel, 2, depthToWorldTransformParameter.getOpenCLBufferObject());
      openCLManager.setKernelArgument(kernel, 3, pointCloudVertexOutput.getOpenCLBufferObject());

      openCLManager.execute2D(kernel, depthImage.getImageWidth(), depthImage.getImageHeight());

      pointCloudVertexOutput.readOpenCLBufferObject(openCLManager);
      FloatBuffer pointCloudBuffer = pointCloudVertexOutput.getBackingDirectFloatBuffer();

      ArrayList<Point3D32> pointCloud = new ArrayList<>();
      for (int i = 0; i < numberOfPixels * FLOATS_PER_PIXEL; i += FLOATS_PER_PIXEL)
      {
         if (pointCloudBuffer.get(i) > 0.0f)
         {
            float x = pointCloudBuffer.get(i + 1);
            float y = pointCloudBuffer.get(i + 2);
            float z = pointCloudBuffer.get(i + 3);
            pointCloud.add(new Point3D32(x, y, z));
         }
      }

      depthImage.release();
      bytedecoDepthImage.destroy(openCLManager);

      return pointCloud;
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      if (pointCloudVertexOutput != null)
         pointCloudVertexOutput.destroy(openCLManager);

      openCLProgram.close();
      kernel.close();
      System.out.println("Destroyed " + getClass().getSimpleName());
   }
}
