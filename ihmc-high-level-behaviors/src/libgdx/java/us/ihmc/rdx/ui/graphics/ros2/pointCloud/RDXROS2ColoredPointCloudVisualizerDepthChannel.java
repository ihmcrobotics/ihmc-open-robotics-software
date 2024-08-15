package us.ihmc.rdx.ui.graphics.ros2.pointCloud;

import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.cuda.CUDACompressionTools;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.SwapReference;

/**
 * Channel used for the depth part. Handles decompression.
 */
public class RDXROS2ColoredPointCloudVisualizerDepthChannel extends RDXROS2ColoredPointCloudVisualizerChannel
{
   private CUDACompressionTools compressionTools;
   private SwapReference<BytedecoImage> depth16UC1ImageSwapReference;

   public RDXROS2ColoredPointCloudVisualizerDepthChannel(ROS2Topic<ImageMessage> topic)
   {
      super("Depth ", topic);
   }

   @Override
   protected void initialize(OpenCLManager openCLManager)
   {
      depth16UC1ImageSwapReference = new SwapReference<>(() ->
      {
         BytedecoImage depth16UC1Image = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);
         depth16UC1Image.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
         getFrequencyText().ping();
         return depth16UC1Image;
      });

      if (ImageMessageFormat.getFormat(imageMessage) == ImageMessageFormat.DEPTH_HYBRID_ZSTD_JPEG_16UC1)
         compressionTools = new CUDACompressionTools();
   }

   @Override
   protected void decompress()
   {
      synchronized (decompressionInputSwapReference)
      {
         if (ImageMessageFormat.getFormat(imageMessage) == ImageMessageFormat.DEPTH_HYBRID_ZSTD_JPEG_16UC1)
         {
            compressionTools.decompressDepth(decompressionInputSwapReference.getForThreadTwo().getInputPointer(),
                                             depth16UC1ImageSwapReference.getForThreadOne().getBytedecoOpenCVMat());
         }
         else
         {
            opencv_imgcodecs.imdecode(decompressionInputSwapReference.getForThreadTwo().getInputMat(),
                                      opencv_imgcodecs.IMREAD_UNCHANGED,
                                      depth16UC1ImageSwapReference.getForThreadOne().getBytedecoOpenCVMat());
         }
      }
      depth16UC1ImageSwapReference.swap();
   }

   @Override
   protected Object getDecompressionAccessSyncObject()
   {
      return depth16UC1ImageSwapReference;
   }

   @Override
   public void destroy()
   {
      super.destroy();
      if (compressionTools != null)
         compressionTools.destroy();
   }

   public BytedecoImage getDepth16UC1Image()
   {
      return depth16UC1ImageSwapReference.getForThreadTwo();
   }
}
