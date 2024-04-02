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

   private final MutableReferenceFrame zedFrame = new MutableReferenceFrame();
   private final RigidBodyTransform zedToRealsenseTransform = new RigidBodyTransform();

   private final MutableReferenceFrame realsenseFrame = new MutableReferenceFrame();
   private final Object imageDataUsageSynchronizer = new Object();

   private RawImage lowQualityImage;
   private BytedecoImage bytedecoLowQualityImage;
   private RawImage highQualityImage;

   private BytedecoImage bytedecoZEDOutput;

   public DepthImageOverlapRemover()
   {
      this.openCLManager = new OpenCLManager();
      openCLProgram = openCLManager.loadProgram("DepthOverlapRemover", "PerceptionCommon.cl");
      kernel = openCLManager.createKernel(openCLProgram, "removeDepthOverlap");
   }

   /**
    * Sets the high quality image (the unmodified image)
    *
    * @param highQualityImage the high quality image
    */
   public void setHighQualityImage(RawImage highQualityImage)
   {
      synchronized (imageDataUsageSynchronizer)
      {
         if (this.highQualityImage != null)
         {
            this.highQualityImage.release();
         }
         this.highQualityImage = highQualityImage;
      }
   }

   /**
    * Removes the overlapping region between the high quality and low quality image from the low quality image.
    * The high quality image will not be modified, while the low quality image will have the overlapping portion cut out.
    *
    * @param lowQualityImage the low quality image which will have the overlapping portion cut out
    * @return A new RawImage object which is the provided low quality image without the overlapping section.
    */
   public RawImage removeOverlap(RawImage lowQualityImage, int allowedOverlap)
   {
      if (this.lowQualityImage != null)
         this.lowQualityImage.release();
      this.lowQualityImage = lowQualityImage;

      synchronized (imageDataUsageSynchronizer)
      {
         if (highQualityImage == null || highQualityImage.isEmpty())
            return this.lowQualityImage;

         zedFrame.update(transformToWorld -> transformToWorld.set(this.lowQualityImage.getOrientation(), this.lowQualityImage.getPosition()));
         realsenseFrame.update(transformToWorld -> transformToWorld.set(highQualityImage.getOrientation(), highQualityImage.getPosition()));
         zedFrame.getReferenceFrame().getTransformToDesiredFrame(zedToRealsenseTransform, realsenseFrame.getReferenceFrame());
         zedToRealsenseTransformParameter.setParameter(zedToRealsenseTransform);
         zedToRealsenseTransformParameter.writeOpenCLBufferObject(openCLManager);

         parameters.setParameter(this.lowQualityImage.getFocalLengthX());
         parameters.setParameter(this.lowQualityImage.getFocalLengthY());
         parameters.setParameter(this.lowQualityImage.getPrincipalPointX());
         parameters.setParameter(this.lowQualityImage.getPrincipalPointY());
         parameters.setParameter(this.lowQualityImage.getDepthDiscretization());
         parameters.setParameter(this.lowQualityImage.getImageWidth());
         parameters.setParameter(this.lowQualityImage.getImageHeight());
         parameters.setParameter(highQualityImage.getFocalLengthX());
         parameters.setParameter(highQualityImage.getFocalLengthY());
         parameters.setParameter(highQualityImage.getPrincipalPointX());
         parameters.setParameter(highQualityImage.getPrincipalPointY());
         parameters.setParameter(highQualityImage.getDepthDiscretization());
         parameters.setParameter(highQualityImage.getImageWidth());
         parameters.setParameter(highQualityImage.getImageHeight());
         parameters.setParameter(allowedOverlap);
         parameters.writeOpenCLBufferObject(openCLManager);
      }

      if (bytedecoLowQualityImage != null)
         bytedecoLowQualityImage.destroy(openCLManager);
      bytedecoLowQualityImage = new BytedecoImage(this.lowQualityImage.getCpuImageMat());
      bytedecoLowQualityImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
      bytedecoLowQualityImage.writeOpenCLImage(openCLManager);

      if (bytedecoZEDOutput != null)
         bytedecoZEDOutput.destroy(openCLManager);
      bytedecoZEDOutput = new BytedecoImage(this.lowQualityImage.getImageWidth(), this.lowQualityImage.getImageHeight(), this.lowQualityImage.getOpenCVType());
      bytedecoZEDOutput.createOpenCLImage(openCLManager, OpenCL.CL_MEM_WRITE_ONLY);

      openCLManager.setKernelArgument(kernel, 0, bytedecoLowQualityImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 1, bytedecoZEDOutput.getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 2, parameters.getOpenCLBufferObject());
      openCLManager.setKernelArgument(kernel, 3, zedToRealsenseTransformParameter.getOpenCLBufferObject());

      openCLManager.execute2D(kernel, this.lowQualityImage.getImageWidth(), this.lowQualityImage.getImageHeight());

      bytedecoZEDOutput.readOpenCLImage(openCLManager);

      lowQualityImage.release();

      return new RawImage(lowQualityImage.getSequenceNumber(),
                          lowQualityImage.getAcquisitionTime(),
                          lowQualityImage.getImageWidth(),
                          lowQualityImage.getImageHeight(),
                          lowQualityImage.getDepthDiscretization(),
                          bytedecoZEDOutput.getBytedecoOpenCVMat(),
                          null,
                          lowQualityImage.getOpenCVType(),
                          lowQualityImage.getFocalLengthX(),
                          lowQualityImage.getFocalLengthY(),
                          lowQualityImage.getPrincipalPointX(),
                          lowQualityImage.getPrincipalPointY(),
                          lowQualityImage.getPosition(),
                          lowQualityImage.getOrientation());
   }
}