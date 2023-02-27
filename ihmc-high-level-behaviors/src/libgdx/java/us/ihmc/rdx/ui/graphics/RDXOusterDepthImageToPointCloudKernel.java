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
import us.ihmc.perception.opencl.OpenCLRigidBodyTransformParameter;
import us.ihmc.rdx.RDXPointCloudRenderer;

public class RDXOusterDepthImageToPointCloudKernel
{
   private RDXPointCloudRenderer pointCloudRenderer;
   private final BytedecoImage depthImage;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel unpackPointCloudKernel;
   private final OpenCLFloatParameters floatParameters = new OpenCLFloatParameters();
   private final OpenCLFloatParameters fisheyeFloatParameters = new OpenCLFloatParameters();
   private final OpenCLRigidBodyTransformParameter ousterToWorldTransformParameter = new OpenCLRigidBodyTransformParameter();
   private final OpenCLRigidBodyTransformParameter ousterToFisheyeTransformParameter = new OpenCLRigidBodyTransformParameter();
   private OpenCLFloatBuffer pointCloudVertexBuffer;
   private final RigidBodyTransform ousterToWorldTransform = new RigidBodyTransform();
   private int levelOfColorDetail;
   private int heightWithVerticalPointsForColorDetail;
   private BytedecoImage fisheyeImage;
   private final RigidBodyTransform ousterToFisheyeTransform = new RigidBodyTransform();
   private double fisheyeFocalLengthPixelsX;
   private double fisheyeFocalLengthPixelsY;
   private double fisheyePrincipalPointPixelsX;
   private double fisheyePrincipalPointPixelsY;

   public RDXOusterDepthImageToPointCloudKernel(RDXPointCloudRenderer pointCloudRenderer, OpenCLManager openCLManager, int depthWidth, int depthHeight)
   {
      depthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
      depthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      initialize(pointCloudRenderer, openCLManager, 0);
   }

   public RDXOusterDepthImageToPointCloudKernel(RDXPointCloudRenderer pointCloudRenderer,
                                                OpenCLManager openCLManager,
                                                BytedecoImage depthImage,
                                                int levelOfColorDetail)
   {
      this.depthImage = depthImage;
      initialize(pointCloudRenderer, openCLManager, levelOfColorDetail);
   }

   private void initialize(RDXPointCloudRenderer pointCloudRenderer,
                           OpenCLManager openCLManager,
                           int pointsToAddAboveAndBelowForColorDetail)
   {
      this.openCLManager = openCLManager;

      openCLProgram = openCLManager.loadProgram("OusterPointCloudVisualizer", "PerceptionCommon.cl");
      unpackPointCloudKernel = openCLManager.createKernel(openCLProgram, "imageToPointCloud");

      changeLevelOfColorDetail(pointCloudRenderer, pointsToAddAboveAndBelowForColorDetail);

      fisheyeImage = new BytedecoImage(100, 100, opencv_core.CV_8UC4);
      fisheyeImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
   }

   public void changeLevelOfColorDetail(RDXPointCloudRenderer pointCloudRenderer, int levelOfColorDetail)
   {
      this.pointCloudRenderer = pointCloudRenderer;
      this.levelOfColorDetail = levelOfColorDetail;
      this.heightWithVerticalPointsForColorDetail = depthImage.getImageHeight() * (1 + 2 * levelOfColorDetail);

      pointCloudVertexBuffer = new OpenCLFloatBuffer(pointCloudRenderer.getMaxPoints() * RDXPointCloudRenderer.FLOATS_PER_VERTEX,
                                                     pointCloudRenderer.getVertexBuffer());
      pointCloudVertexBuffer.createOpenCLBufferObject(openCLManager);
   }

   public void updateSensorTransform(ReferenceFrame sensorFrame)
   {
      sensorFrame.getTransformToDesiredFrame(ousterToWorldTransform, ReferenceFrame.getWorldFrame());
   }

   public void setFisheyeImageToColorPoints(BytedecoImage fThetaFisheyeRGBA8Image,
                                            double fisheyeFocalLengthPixelsX,
                                            double fisheyeFocalLengthPixelsY,
                                            double fisheyePrincipalPointPixelsX,
                                            double fisheyePrincipalPointPixelsY)
   {
      this.fisheyeFocalLengthPixelsX = fisheyeFocalLengthPixelsX;
      this.fisheyeFocalLengthPixelsY = fisheyeFocalLengthPixelsY;
      this.fisheyePrincipalPointPixelsX = fisheyePrincipalPointPixelsX;
      this.fisheyePrincipalPointPixelsY = fisheyePrincipalPointPixelsY;

      fisheyeImage.ensureDimensionsMatch(fThetaFisheyeRGBA8Image, openCLManager);
      fThetaFisheyeRGBA8Image.getBytedecoOpenCVMat().copyTo(fisheyeImage.getBytedecoOpenCVMat());
   }

   public RigidBodyTransform getOusterToFisheyeTransformToPack()
   {
      return ousterToFisheyeTransform;
   }

   public void runKernel(float horizontalFieldOfView, float verticalFieldOfView, float pointSize)
   {
      boolean useFisheyeColorImage = fisheyeImage.getImageWidth() > 100;

      floatParameters.setParameter(horizontalFieldOfView);
      floatParameters.setParameter(verticalFieldOfView);
      floatParameters.setParameter(depthImage.getImageWidth());
      floatParameters.setParameter(depthImage.getImageHeight());
      floatParameters.setParameter(pointSize);
      floatParameters.setParameter((float) levelOfColorDetail);
      ousterToWorldTransformParameter.setParameter(ousterToWorldTransform);

      fisheyeFloatParameters.setParameter(fisheyeImage.getImageWidth());
      fisheyeFloatParameters.setParameter(fisheyeImage.getImageHeight());
      fisheyeFloatParameters.setParameter((float) fisheyeFocalLengthPixelsX);
      fisheyeFloatParameters.setParameter((float) fisheyeFocalLengthPixelsY);
      fisheyeFloatParameters.setParameter((float) fisheyePrincipalPointPixelsX);
      fisheyeFloatParameters.setParameter((float) fisheyePrincipalPointPixelsY);
      fisheyeFloatParameters.setParameter(useFisheyeColorImage);
      ousterToFisheyeTransformParameter.setParameter(ousterToFisheyeTransform);

      floatParameters.writeOpenCLBufferObject(openCLManager);
      ousterToWorldTransformParameter.writeOpenCLBufferObject(openCLManager);
      fisheyeFloatParameters.writeOpenCLBufferObject(openCLManager);
      ousterToFisheyeTransformParameter.writeOpenCLBufferObject(openCLManager);
      depthImage.writeOpenCLImage(openCLManager);
      if (useFisheyeColorImage)
         fisheyeImage.writeOpenCLImage(openCLManager);
      pointCloudRenderer.updateMeshFastestBeforeKernel();
      pointCloudVertexBuffer.syncWithBackingBuffer();

      openCLManager.setKernelArgument(unpackPointCloudKernel, 0, floatParameters.getOpenCLBufferObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 1, ousterToWorldTransformParameter.getOpenCLBufferObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 2, depthImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 3, fisheyeFloatParameters.getOpenCLBufferObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 4, fisheyeImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 5, ousterToFisheyeTransformParameter.getOpenCLBufferObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 6, pointCloudVertexBuffer.getOpenCLBufferObject());
      openCLManager.execute2D(unpackPointCloudKernel, depthImage.getImageWidth(), heightWithVerticalPointsForColorDetail);
      pointCloudVertexBuffer.readOpenCLBufferObject(openCLManager);

      pointCloudRenderer.updateMeshFastestAfterKernel();
   }

   public BytedecoImage getDepthImage()
   {
      return depthImage;
   }

   public RigidBodyTransform getOusterToWorldTransform()
   {
      return ousterToWorldTransform;
   }
}
