package us.ihmc;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencl.OpenCLRigidBodyTransformParameter;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

public class DepthImageOverlapRemover
{
   private final OpenCLManager openCLManager;
   private final _cl_program openCLProgram;
   private final _cl_kernel kernel;
   private final OpenCLFloatParameters parameters = new OpenCLFloatParameters();
   private final OpenCLRigidBodyTransformParameter zedToRealsenseTransformParameter = new OpenCLRigidBodyTransformParameter();
   private final OpenCLRigidBodyTransformParameter realsenseToWorldTransformParameter = new OpenCLRigidBodyTransformParameter();

   private final MutableReferenceFrame zedFrame = new MutableReferenceFrame();
   private final RigidBodyTransform zedToRealsenseTransform = new RigidBodyTransform();

   private final MutableReferenceFrame realsenseFrame = new MutableReferenceFrame();

   private RawImage inputZEDImage;
   private BytedecoImage bytedecoZEDInput;
   private RawImage inputRealsenseImage;
   private BytedecoImage bytedecoRealsenseInput;

   private BytedecoImage bytedecoZEDOutput;

   public DepthImageOverlapRemover()
   {
      this.openCLManager = new OpenCLManager();
      openCLProgram = openCLManager.loadProgram("DepthOverlapRemover", "PerceptionCommon.cl");
      kernel = openCLManager.createKernel(openCLProgram, "removeDepthOverlap");
   }

   public void setRealsenseDepthImage(RawImage realsenseDepthImage)
   {
      if (inputRealsenseImage != null)
      {
         inputRealsenseImage.release();
         bytedecoRealsenseInput.destroy(openCLManager);
      }
      inputRealsenseImage = realsenseDepthImage;
      bytedecoRealsenseInput = new BytedecoImage(inputRealsenseImage.getCpuImageMatrix());
   }

   public RawImage removeOverlap(RawImage realsenseDepthImage, RawImage zedDepthImage)
   {
      if (inputZEDImage != null)
         inputZEDImage.release();
      inputZEDImage = zedDepthImage;

      if (inputRealsenseImage != null)
         inputRealsenseImage.release();
      inputRealsenseImage = realsenseDepthImage;

      if (inputRealsenseImage == null || inputRealsenseImage.isEmpty())
         return inputZEDImage;

      zedFrame.update(transformToWorld -> transformToWorld.set(inputZEDImage.getOrientation(), inputZEDImage.getPosition()));
      realsenseFrame.update(transformToWorld -> transformToWorld.set(inputRealsenseImage.getOrientation(), inputRealsenseImage.getPosition()));
      zedFrame.getReferenceFrame().getTransformToDesiredFrame(zedToRealsenseTransform, realsenseFrame.getReferenceFrame());
      zedToRealsenseTransformParameter.setParameter(zedToRealsenseTransform);
      zedToRealsenseTransformParameter.writeOpenCLBufferObject(openCLManager);

      parameters.setParameter(inputZEDImage.getFocalLengthX());
      parameters.setParameter(inputZEDImage.getFocalLengthY());
      parameters.setParameter(inputZEDImage.getPrincipalPointX());
      parameters.setParameter(inputZEDImage.getPrincipalPointY());
      parameters.setParameter(inputZEDImage.getDepthDiscretization());
      parameters.setParameter(inputZEDImage.getImageWidth());
      parameters.setParameter(inputZEDImage.getImageHeight());
      parameters.setParameter(inputRealsenseImage.getFocalLengthX());
      parameters.setParameter(inputRealsenseImage.getFocalLengthY());
      parameters.setParameter(inputRealsenseImage.getPrincipalPointX());
      parameters.setParameter(inputRealsenseImage.getPrincipalPointY());
      parameters.setParameter(inputRealsenseImage.getDepthDiscretization());
      parameters.setParameter(inputRealsenseImage.getImageWidth());
      parameters.setParameter(inputRealsenseImage.getImageHeight());
      parameters.writeOpenCLBufferObject(openCLManager);

      if (bytedecoZEDInput != null)
         bytedecoZEDInput.destroy(openCLManager);
      bytedecoZEDInput = new BytedecoImage(inputZEDImage.getCpuImageMatrix());
      bytedecoZEDInput.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
      bytedecoZEDInput.writeOpenCLImage(openCLManager);

      if (bytedecoRealsenseInput != null)
         bytedecoRealsenseInput.destroy(openCLManager);
      bytedecoRealsenseInput = new BytedecoImage(inputRealsenseImage.getCpuImageMatrix());
      bytedecoRealsenseInput.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
      bytedecoRealsenseInput.writeOpenCLImage(openCLManager);

      if (bytedecoZEDOutput != null)
         bytedecoZEDOutput.destroy(openCLManager);
      bytedecoZEDOutput = new BytedecoImage(inputZEDImage.getImageWidth(), inputZEDImage.getImageHeight(), inputZEDImage.getOpenCVType());
      bytedecoZEDOutput.createOpenCLImage(openCLManager, OpenCL.CL_MEM_WRITE_ONLY);

      openCLManager.setKernelArgument(kernel, 0, bytedecoZEDInput.getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 1, bytedecoRealsenseInput.getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 2, bytedecoZEDOutput.getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 3, parameters.getOpenCLBufferObject());
      openCLManager.setKernelArgument(kernel, 4, zedToRealsenseTransformParameter.getOpenCLBufferObject());

      openCLManager.execute2D(kernel, inputZEDImage.getImageWidth(), inputZEDImage.getImageHeight());

      bytedecoZEDOutput.readOpenCLImage(openCLManager);

      zedDepthImage.release();

      return new RawImage(zedDepthImage.getSequenceNumber(),
                          zedDepthImage.getAcquisitionTime(),
                          zedDepthImage.getImageWidth(),
                          zedDepthImage.getImageHeight(),
                          zedDepthImage.getDepthDiscretization(),
                          bytedecoZEDOutput.getBytedecoOpenCVMat(),
                          null,
                          zedDepthImage.getOpenCVType(),
                          zedDepthImage.getFocalLengthX(),
                          zedDepthImage.getFocalLengthY(),
                          zedDepthImage.getPrincipalPointX(),
                          zedDepthImage.getPrincipalPointY(),
                          zedDepthImage.getPosition(),
                          zedDepthImage.getOrientation());
   }
}