package us.ihmc.perception.ouster;

import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opencl.OpenCLManager;

import java.util.function.Supplier;

public class OusterDepthImageRetriever
{
   private long depthSequenceNumber = 0L;

   private RawImage depthImage = null;

   private final Supplier<ReferenceFrame> sensorFrameSupplier;
   private final FramePose3D sensorFramePose = new FramePose3D();

   private final OusterNetServer ouster;

   private OpenCLManager openCLManager;
   private OusterDepthExtractionKernel depthExtractionKernel;

   private boolean running = false;
   private final Supplier<Boolean> computeLidarScan;
   private final Supplier<Boolean> computeHeightMap;

   public OusterDepthImageRetriever(OusterNetServer ouster,
                                    Supplier<ReferenceFrame> sensorFrameSupplier,
                                    Supplier<Boolean> computeLidarScan,
                                    Supplier<Boolean> computeHeightMap)
   {
      this.ouster = ouster;
      this.sensorFrameSupplier = sensorFrameSupplier;
      this.computeLidarScan = computeLidarScan;
      this.computeHeightMap = computeHeightMap;

      openCLManager = new OpenCLManager();
      this.ouster.setOnFrameReceived(this::retrieveDepth);
   }

   private void retrieveDepth()
   {
      if (depthExtractionKernel == null)
         depthExtractionKernel = new OusterDepthExtractionKernel(ouster, openCLManager, computeLidarScan, computeHeightMap);

      if (ouster.isInitialized() && running)
      {
         sensorFramePose.setToZero(sensorFrameSupplier.get());
         sensorFramePose.changeFrame(ReferenceFrame.getWorldFrame());

         // grab the image
         depthExtractionKernel.copyLidarFrameBuffer();
         depthExtractionKernel.runKernel(sensorFramePose.getReferenceFrame().getTransformToWorldFrame());

         depthImage = new RawImage(depthSequenceNumber++,
                                   ouster.getAquisitionInstant(),
                                   ouster.getImageWidth(),
                                   ouster.getImageHeight(),
                                   1.0f,
                                   depthExtractionKernel.getExtractedDepthImage().getBytedecoOpenCVMat().clone(),
                                   null,
                                   opencv_core.CV_16UC1,
                                   ouster.getImageWidth() / (2.0f * (float) Math.PI),
                                   // These are nominal values approximated by Duncan & Tomasz
                                   ouster.getImageHeight() / ((float) Math.PI / 2.0f),
                                   0,
                                   0,
                                   sensorFramePose.getPosition(),
                                   sensorFramePose.getOrientation());
      }
   }

   public RawImage getLatestRawDepthImage()
   {
      return depthImage;
   }

   public void start()
   {
      running = true;
   }

   public void stop()
   {
      running = false;
   }

   public void destroy()
   {
      stop();
      depthImage.destroy();
   }
}
