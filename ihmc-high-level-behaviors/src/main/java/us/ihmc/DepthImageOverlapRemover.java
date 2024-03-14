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

   private RawImage slaveImage;
   private BytedecoImage bytedecoSlaveImage;
   private RawImage masterImage;

   private BytedecoImage bytedecoZEDOutput;

   public DepthImageOverlapRemover()
   {
      this.openCLManager = new OpenCLManager();
      openCLProgram = openCLManager.loadProgram("DepthOverlapRemover", "PerceptionCommon.cl");
      kernel = openCLManager.createKernel(openCLProgram, "removeDepthOverlap");
   }

   /**
    * Sets the master image (the unmodified image)
    *
    * @param masterImage the master image
    */
   public void setMasterImage(RawImage masterImage)
   {
      synchronized (imageDataUsageSynchronizer)
      {
         if (this.masterImage != null)
         {
            this.masterImage.release();
         }
         this.masterImage = masterImage;
      }
   }

   /**
    * Removes the overlapping region between the master and slave image from the slave image.
    * The master image will not be modified, while the slave image will have the overlapping portion cut out.
    *
    * @param slaveImage the slave image which will have the overlapping portion cut out
    * @return A new RawImage object which is the provided slave image without the overlapping section.
    */
   public RawImage removeOverlap(RawImage slaveImage)
   {
      if (this.slaveImage != null)
         this.slaveImage.release();
      this.slaveImage = slaveImage;

      synchronized (imageDataUsageSynchronizer)
      {
         if (masterImage == null || masterImage.isEmpty())
            return this.slaveImage;

         zedFrame.update(transformToWorld -> transformToWorld.set(this.slaveImage.getOrientation(), this.slaveImage.getPosition()));
         realsenseFrame.update(transformToWorld -> transformToWorld.set(masterImage.getOrientation(), masterImage.getPosition()));
         zedFrame.getReferenceFrame().getTransformToDesiredFrame(zedToRealsenseTransform, realsenseFrame.getReferenceFrame());
         zedToRealsenseTransformParameter.setParameter(zedToRealsenseTransform);
         zedToRealsenseTransformParameter.writeOpenCLBufferObject(openCLManager);

         parameters.setParameter(this.slaveImage.getFocalLengthX());
         parameters.setParameter(this.slaveImage.getFocalLengthY());
         parameters.setParameter(this.slaveImage.getPrincipalPointX());
         parameters.setParameter(this.slaveImage.getPrincipalPointY());
         parameters.setParameter(this.slaveImage.getDepthDiscretization());
         parameters.setParameter(this.slaveImage.getImageWidth());
         parameters.setParameter(this.slaveImage.getImageHeight());
         parameters.setParameter(masterImage.getFocalLengthX());
         parameters.setParameter(masterImage.getFocalLengthY());
         parameters.setParameter(masterImage.getPrincipalPointX());
         parameters.setParameter(masterImage.getPrincipalPointY());
         parameters.setParameter(masterImage.getDepthDiscretization());
         parameters.setParameter(masterImage.getImageWidth());
         parameters.setParameter(masterImage.getImageHeight());
         parameters.writeOpenCLBufferObject(openCLManager);
      }

      if (bytedecoSlaveImage != null)
         bytedecoSlaveImage.destroy(openCLManager);
      bytedecoSlaveImage = new BytedecoImage(this.slaveImage.getCpuImageMat());
      bytedecoSlaveImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
      bytedecoSlaveImage.writeOpenCLImage(openCLManager);

      if (bytedecoZEDOutput != null)
         bytedecoZEDOutput.destroy(openCLManager);
      bytedecoZEDOutput = new BytedecoImage(this.slaveImage.getImageWidth(), this.slaveImage.getImageHeight(), this.slaveImage.getOpenCVType());
      bytedecoZEDOutput.createOpenCLImage(openCLManager, OpenCL.CL_MEM_WRITE_ONLY);

      openCLManager.setKernelArgument(kernel, 0, bytedecoSlaveImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 1, bytedecoZEDOutput.getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 2, parameters.getOpenCLBufferObject());
      openCLManager.setKernelArgument(kernel, 3, zedToRealsenseTransformParameter.getOpenCLBufferObject());

      openCLManager.execute2D(kernel, this.slaveImage.getImageWidth(), this.slaveImage.getImageHeight());

      bytedecoZEDOutput.readOpenCLImage(openCLManager);

      slaveImage.release();

      return new RawImage(slaveImage.getSequenceNumber(),
                          slaveImage.getAcquisitionTime(),
                          slaveImage.getImageWidth(),
                          slaveImage.getImageHeight(),
                          slaveImage.getDepthDiscretization(),
                          bytedecoZEDOutput.getBytedecoOpenCVMat(),
                          null,
                          slaveImage.getOpenCVType(),
                          slaveImage.getFocalLengthX(),
                          slaveImage.getFocalLengthY(),
                          slaveImage.getPrincipalPointX(),
                          slaveImage.getPrincipalPointY(),
                          slaveImage.getPosition(),
                          slaveImage.getOrientation());
   }
}