package us.ihmc.rdx.ui.graphics.ros2.pointCloud;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.cuda.CUDAImageEncoder;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.SwapReference;

/**
 * Channel used for the color part. Handles decompression and conversion.
 */
public class RDXROS2ColoredPointCloudVisualizerColorChannel extends RDXROS2ColoredPointCloudVisualizerChannel
{
   private CUDAImageEncoder jpegDecoder = new CUDAImageEncoder();
   private Mat imageFromMessage;
   private SwapReference<BytedecoImage> color8UC4ImageSwapReference;

   public RDXROS2ColoredPointCloudVisualizerColorChannel(ROS2Topic<ImageMessage> topic)
   {
      super("Color ", topic);
   }

   @Override
   protected void initialize(OpenCLManager openCLManager)
   {
      switch (ImageMessageFormat.getFormat(imageMessage))
      {
         case COLOR_JPEG_YUVI420 -> imageFromMessage = new Mat(1, 1, opencv_core.CV_8UC1);
         case COLOR_JPEG_BGR8 -> imageFromMessage = new Mat(1, 1, opencv_core.CV_8UC3);
         default -> LogTools.error("Visualization attempted using unimplemented color format.");
      }
      color8UC4ImageSwapReference = new SwapReference<>(() ->
      {
         BytedecoImage color8UC4Image = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC4);
         color8UC4Image.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
         getFrequencyText().ping();
         return color8UC4Image;
      });
   }

   @Override
   protected void decompress()
   {
      synchronized (decompressionInputSwapReference)
      {
         BytePointer decompressionInputData = decompressionInputSwapReference.getForThreadTwo().getInputBytePointer();
         jpegDecoder.decodeToBGR(decompressionInputData, decompressionInputData.limit(), imageFromMessage);
      }

      opencv_imgproc.cvtColor(imageFromMessage, color8UC4ImageSwapReference.getForThreadOne().getBytedecoOpenCVMat(), opencv_imgproc.COLOR_BGR2RGBA);

      switch (ImageMessageFormat.getFormat(imageMessage))
      {
         case COLOR_JPEG_YUVI420 ->
         {
            opencv_imgproc.cvtColor(imageFromMessage, color8UC4ImageSwapReference.getForThreadOne().getBytedecoOpenCVMat(), opencv_imgproc.COLOR_YUV2RGBA_I420);
         }
         case COLOR_JPEG_BGR8 ->
         {
            opencv_imgproc.cvtColor(imageFromMessage, color8UC4ImageSwapReference.getForThreadOne().getBytedecoOpenCVMat(), opencv_imgproc.COLOR_BGR2RGBA);
         }
         default -> LogTools.error("Visualization attempted using unimplemented color format.");
      }

      color8UC4ImageSwapReference.swap();
   }

   @Override
   protected Object getDecompressionAccessSyncObject()
   {
      return color8UC4ImageSwapReference;
   }

   public BytedecoImage getColor8UC4Image()
   {
      return color8UC4ImageSwapReference.getForThreadTwo();
   }
}
