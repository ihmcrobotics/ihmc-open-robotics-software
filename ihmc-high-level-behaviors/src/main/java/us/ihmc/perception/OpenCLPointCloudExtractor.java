package us.ihmc.perception;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencl.OpenCLRigidBodyTransformParameter;

import java.nio.FloatBuffer;

/**
 * Class used to get a RecyclingArrayList of Point3D32s (point cloud) from a 16UC1 depth image
 */
public class OpenCLPointCloudExtractor
{
   private static final int FLOATS_PER_PIXEL = 3;
   private final OpenCLManager openCLManager;
   private final _cl_program openCLProgram;
   private final _cl_kernel kernel;
   private final OpenCLFloatParameters parametersBuffer = new OpenCLFloatParameters();
   private final OpenCLRigidBodyTransformParameter depthToWorldTransformParameter = new OpenCLRigidBodyTransformParameter();
   private OpenCLFloatBuffer pointCloudVertexOutput;

   private RawImage depthImage;
   private BytedecoImage bytedecoDepthImage;

   public OpenCLPointCloudExtractor(OpenCLManager openCLManager)
   {
      this.openCLManager = openCLManager;
      openCLProgram = openCLManager.loadProgram("DepthImageToPointCloudConverter", "PerceptionCommon.cl");
      kernel = openCLManager.createKernel(openCLProgram, "convertDepthImageToPointCloud");
   }

   public RecyclingArrayList<Point3D32> extractPointCloud(RawImage depthImage16UC1)
   {
      if (depthImage != null)
         depthImage.release();
      depthImage = depthImage16UC1.get();

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

      if (bytedecoDepthImage != null)
         bytedecoDepthImage.destroy(openCLManager);
      bytedecoDepthImage = new BytedecoImage(depthImage.getCpuImageMatrix());
      bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
      bytedecoDepthImage.writeOpenCLImage(openCLManager);

      openCLManager.setKernelArgument(kernel, 0, bytedecoDepthImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 1, parametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(kernel, 2, depthToWorldTransformParameter.getOpenCLBufferObject());
      openCLManager.setKernelArgument(kernel, 3, pointCloudVertexOutput.getOpenCLBufferObject());

      openCLManager.execute2D(kernel, depthImage.getImageWidth(), depthImage.getImageHeight());

      pointCloudVertexOutput.readOpenCLBufferObject(openCLManager);
      FloatBuffer pointCloudBuffer = pointCloudVertexOutput.getBackingDirectFloatBuffer();

      RecyclingArrayList<Point3D32> pointCloud = new RecyclingArrayList<>(Point3D32::new);
      for (int i = 0; i < numberOfPixels * FLOATS_PER_PIXEL; i += FLOATS_PER_PIXEL)
      {
         float x = pointCloudBuffer.get(i + 0);
         float y = pointCloudBuffer.get(i + 1);
         float z = pointCloudBuffer.get(i + 2);
         Point3D32 addedPoint = pointCloud.add();
         addedPoint.set(x, y, z);
      }

      return pointCloud;
   }

   public void destroy()
   {
      if (bytedecoDepthImage != null)
         bytedecoDepthImage.destroy(openCLManager);
      if (depthImage != null)
         depthImage.release();

      openCLProgram.close();
      kernel.close();

      openCLManager.destroy();
   }
}
