package us.ihmc.rdx.ui.graphics;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.rdx.RDXPointCloudRenderer;

public class RDXOusterDepthImageToPointCloudKernel
{
   private RDXPointCloudRenderer pointCloudRenderer;
   private final BytedecoImage depthImage;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel unpackPointCloudKernel;
   private final OpenCLFloatParameters parametersOpenCLFloatBuffer = new OpenCLFloatParameters();
   private OpenCLFloatBuffer pointCloudVertexBuffer;
   private final RigidBodyTransform sensorTransformToWorld = new RigidBodyTransform();

   public RDXOusterDepthImageToPointCloudKernel(RDXPointCloudRenderer pointCloudRenderer, OpenCLManager openCLManager, int depthWidth, int depthHeight)
   {
      depthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
      depthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      initialize(pointCloudRenderer, openCLManager);
   }

   public RDXOusterDepthImageToPointCloudKernel(RDXPointCloudRenderer pointCloudRenderer, OpenCLManager openCLManager, BytedecoImage depthImage)
   {
      this.depthImage = depthImage;
      initialize(pointCloudRenderer, openCLManager);
   }

   private void initialize(RDXPointCloudRenderer pointCloudRenderer, OpenCLManager openCLManager)
   {
      this.pointCloudRenderer = pointCloudRenderer;
      this.openCLManager = openCLManager;

      openCLProgram = openCLManager.loadProgram("OusterPointCloudVisualizer", "PerceptionCommon.cl");
      unpackPointCloudKernel = openCLManager.createKernel(openCLProgram, "imageToPointCloud");

      int totalNumberOfPoints = depthImage.getImageWidth() * depthImage.getImageHeight();
      pointCloudVertexBuffer = new OpenCLFloatBuffer(totalNumberOfPoints * RDXPointCloudRenderer.FLOATS_PER_VERTEX,
                                                     pointCloudRenderer.getVertexBuffer());
      pointCloudVertexBuffer.createOpenCLBufferObject(openCLManager);
   }

   public void updateSensorTransform(ReferenceFrame sensorFrame)
   {
      sensorFrame.getTransformToDesiredFrame(sensorTransformToWorld, ReferenceFrame.getWorldFrame());
   }

   public void runKernel(float horizontalFieldOfView,
                         float verticalFieldOfView,
                         float pointSize)
   {
      parametersOpenCLFloatBuffer.setParameter(horizontalFieldOfView);
      parametersOpenCLFloatBuffer.setParameter(verticalFieldOfView);
      parametersOpenCLFloatBuffer.setParameter(sensorTransformToWorld);
      parametersOpenCLFloatBuffer.setParameter(depthImage.getImageWidth());
      parametersOpenCLFloatBuffer.setParameter(depthImage.getImageHeight());
      parametersOpenCLFloatBuffer.setParameter(pointSize);

      parametersOpenCLFloatBuffer.writeOpenCLBufferObject(openCLManager);
      depthImage.writeOpenCLImage(openCLManager);
      pointCloudRenderer.updateMeshFastestBeforeKernel();
      pointCloudVertexBuffer.syncWithBackingBuffer();

      openCLManager.setKernelArgument(unpackPointCloudKernel, 0, parametersOpenCLFloatBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 1, depthImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 2, pointCloudVertexBuffer.getOpenCLBufferObject());
      openCLManager.execute2D(unpackPointCloudKernel, depthImage.getImageWidth(), depthImage.getImageHeight());
      pointCloudVertexBuffer.readOpenCLBufferObject(openCLManager);

      pointCloudRenderer.updateMeshFastestAfterKernel();
   }

   public BytedecoImage getDepthImage()
   {
      return depthImage;
   }

   public RigidBodyTransform getSensorTransformToWorld()
   {
      return sensorTransformToWorld;
   }
}
