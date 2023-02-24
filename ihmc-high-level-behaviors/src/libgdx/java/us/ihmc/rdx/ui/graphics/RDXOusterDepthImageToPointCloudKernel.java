package us.ihmc.rdx.ui.graphics;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.opencl.OpenCLRigidBodyTransformParameter;

public class RDXOusterDepthImageToPointCloudKernel
{
   private final OpenCLManager openCLManager;
   private final _cl_program openCLProgram;
   private final _cl_kernel unpackPointCloudKernel;
   private final OpenCLFloatParameters floatParameters = new OpenCLFloatParameters();
   private final OpenCLFloatParameters fisheyeFloatParameters = new OpenCLFloatParameters();
   private final OpenCLRigidBodyTransformParameter ousterToWorldTransformParameter = new OpenCLRigidBodyTransformParameter();
   private final OpenCLRigidBodyTransformParameter ousterToFisheyeTransformParameter = new OpenCLRigidBodyTransformParameter();
   private final RigidBodyTransform ousterToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform ousterToFisheyeTransform = new RigidBodyTransform();
   private int levelOfColorDetail;
   private int heightWithVerticalPointsForColorDetail;

   public RDXOusterDepthImageToPointCloudKernel(OpenCLManager openCLManager)
   {
      this.openCLManager = openCLManager;

      openCLProgram = openCLManager.loadProgram("OusterPointCloudVisualizer", "PerceptionCommon.cl");
      unpackPointCloudKernel = openCLManager.createKernel(openCLProgram, "imageToPointCloud");
   }

   public int calculateNumberOfPointsForLevelOfColorDetail(int ousterImageWidth, int ousterImageHeight, int levelOfColorDetail)
   {
      int totalVerticalPointsForColorDetail = 1 + 2 * levelOfColorDetail;
      int heightWithVerticalPointsForColorDetail = ousterImageHeight * totalVerticalPointsForColorDetail;
      this.levelOfColorDetail = levelOfColorDetail;
      this.heightWithVerticalPointsForColorDetail = ousterImageHeight * (1 + 2 * levelOfColorDetail);
      return ousterImageWidth * heightWithVerticalPointsForColorDetail;
   }

   public void updateSensorTransform(ReferenceFrame sensorFrame)
   {
      sensorFrame.getTransformToDesiredFrame(ousterToWorldTransform, ReferenceFrame.getWorldFrame());
   }

   public RigidBodyTransform getOusterToWorldTransformToPack()
   {
      return ousterToWorldTransform;
   }

   public RigidBodyTransform getOusterToFisheyeTransformToPack()
   {
      return ousterToFisheyeTransform;
   }

   public void runKernel(float horizontalFieldOfView,
                         float verticalFieldOfView,
                         float pointSize,
                         BytedecoImage ousterDepthImage,
                         OpenCLFloatBuffer pointCloudVertexBuffer)
   {
      runKernel(horizontalFieldOfView, verticalFieldOfView, pointSize, ousterDepthImage, 0.0, 0.0, 0.0, 0.0, null, pointCloudVertexBuffer);
   }

   public void runKernel(float horizontalFieldOfView,
                         float verticalFieldOfView,
                         float pointSize,
                         BytedecoImage ousterDepthImage,
                         double fisheyeFocalLengthPixelsX,
                         double fisheyeFocalLengthPixelsY,
                         double fisheyePrincipalPointPixelsX,
                         double fisheyePrincipalPointPixelsY,
                         BytedecoImage fisheyeImage,
                         OpenCLFloatBuffer pointCloudVertexBuffer)
   {
      floatParameters.setParameter(horizontalFieldOfView);
      floatParameters.setParameter(verticalFieldOfView);
      floatParameters.setParameter(ousterDepthImage.getImageWidth());
      floatParameters.setParameter(ousterDepthImage.getImageHeight());
      floatParameters.setParameter(pointSize);
      floatParameters.setParameter((float) levelOfColorDetail);
      ousterToWorldTransformParameter.setParameter(ousterToWorldTransform);

      fisheyeFloatParameters.setParameter(fisheyeImage != null ? fisheyeImage.getImageWidth() : 0);
      fisheyeFloatParameters.setParameter(fisheyeImage != null ? fisheyeImage.getImageHeight() : 0);
      fisheyeFloatParameters.setParameter((float) fisheyeFocalLengthPixelsX);
      fisheyeFloatParameters.setParameter((float) fisheyeFocalLengthPixelsY);
      fisheyeFloatParameters.setParameter((float) fisheyePrincipalPointPixelsX);
      fisheyeFloatParameters.setParameter((float) fisheyePrincipalPointPixelsY);
      fisheyeFloatParameters.setParameter(fisheyeImage != null);
      ousterToFisheyeTransformParameter.setParameter(ousterToFisheyeTransform);

      floatParameters.writeOpenCLBufferObject(openCLManager);
      ousterToWorldTransformParameter.writeOpenCLBufferObject(openCLManager);
      fisheyeFloatParameters.writeOpenCLBufferObject(openCLManager);
      ousterToFisheyeTransformParameter.writeOpenCLBufferObject(openCLManager);
      ousterDepthImage.writeOpenCLImage(openCLManager);
      if (fisheyeImage != null)
         fisheyeImage.writeOpenCLImage(openCLManager);

      openCLManager.setKernelArgument(unpackPointCloudKernel, 0, floatParameters.getOpenCLBufferObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 1, ousterToWorldTransformParameter.getOpenCLBufferObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 2, ousterDepthImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 3, fisheyeFloatParameters.getOpenCLBufferObject());
      if (fisheyeImage != null)
         openCLManager.setKernelArgument(unpackPointCloudKernel, 4, fisheyeImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 5, ousterToFisheyeTransformParameter.getOpenCLBufferObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 6, pointCloudVertexBuffer.getOpenCLBufferObject());
      openCLManager.execute2D(unpackPointCloudKernel, ousterDepthImage.getImageWidth(), heightWithVerticalPointsForColorDetail);
      pointCloudVertexBuffer.readOpenCLBufferObject(openCLManager);
   }
}
