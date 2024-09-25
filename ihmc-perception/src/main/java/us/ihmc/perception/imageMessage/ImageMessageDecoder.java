package us.ihmc.perception.imageMessage;

import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.perception.cuda.CUDACompressionTools;
import us.ihmc.perception.cuda.CUDAJPEGProcessor;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.opencv.OpenCVTools;

// TODO: Write tests
public class ImageMessageDecoder
{
   private final ImageMessageDecompressionInput messageDataExtractor = new ImageMessageDecompressionInput();

   private CUDACompressionTools cudaCompressionTools = null;
   private CUDAJPEGProcessor cudaJpegDecoder = null;

   private PixelFormat lastImagePixelFormat = null;

   public ImageMessageDecoder()
   {
      if (CUDATools.hasCUDADevice())
      {
         cudaCompressionTools = new CUDACompressionTools();
         cudaJpegDecoder = new CUDAJPEGProcessor();
      }
   }

   public void decodeMessage(ImageMessage messageToDecode, Mat imageToPack)
   {
      resizeToMessageDimensions(messageToDecode, imageToPack);
      messageDataExtractor.extract(messageToDecode);

      lastImagePixelFormat = PixelFormat.fromImageMessage(messageToDecode);

      switch (CompressionType.fromImageMessage(messageToDecode))
      {
         case JPEG, PNG -> opencv_imgcodecs.imdecode(messageDataExtractor.getInputMat(), opencv_imgcodecs.IMREAD_UNCHANGED, imageToPack);
         case ZSTD_JPEG_HYBRID ->
         {
            if (cudaCompressionTools != null)
               cudaCompressionTools.decompressDepth(messageDataExtractor.getInputPointer(), imageToPack);
         }
         case UNCOMPRESSED -> imageToPack.data(messageDataExtractor.getInputPointer());
      }
   }

   public void decodeMessage(ImageMessage messageToDecode, GpuMat imageToPack)
   {
      resizeToMessageDimensions(messageToDecode, imageToPack);
      messageDataExtractor.extract(messageToDecode);

      lastImagePixelFormat = PixelFormat.fromImageMessage(messageToDecode);

      switch (CompressionType.fromImageMessage(messageToDecode))
      {
         case JPEG ->
         {
            cudaJpegDecoder.decodeToBGR(messageDataExtractor.getInputPointer(), messageDataExtractor.getInputPointer().limit(), imageToPack);
            lastImagePixelFormat = PixelFormat.BGR8;
         }
         case PNG ->
         {
            Mat decompressedImage = new Mat();
            opencv_imgcodecs.imdecode(messageDataExtractor.getInputMat(), opencv_imgcodecs.IMREAD_UNCHANGED, decompressedImage);
            imageToPack.upload(decompressedImage);
            decompressedImage.close();
         }
         case ZSTD_JPEG_HYBRID -> cudaCompressionTools.decompressDepth(messageDataExtractor.getInputPointer(), imageToPack);
         case UNCOMPRESSED ->
         {
            Mat cpuImage = new Mat(imageToPack.size(), imageToPack.type(), messageDataExtractor.getInputPointer());
            imageToPack.upload(cpuImage);
            cpuImage.close();
         }
      }
   }

   public void decodeMessageToRGBA(ImageMessage messageToDecode, Mat imageToPack)
   {
      decodeMessage(messageToDecode, imageToPack);

      switch (lastImagePixelFormat)
      {
         case YUVI420 -> opencv_imgproc.cvtColor(imageToPack, imageToPack, opencv_imgproc.COLOR_YUV2RGBA_I420);
         case BGR8 -> opencv_imgproc.cvtColor(imageToPack, imageToPack, opencv_imgproc.COLOR_BGR2RGBA);
         case BGRA8 -> opencv_imgproc.cvtColor(imageToPack, imageToPack, opencv_imgproc.COLOR_BGRA2RGBA);
         case RGB8 -> opencv_imgproc.cvtColor(imageToPack, imageToPack, opencv_imgproc.COLOR_RGB2RGBA);
         case GRAY8 -> opencv_imgproc.cvtColor(imageToPack, imageToPack, opencv_imgproc.COLOR_GRAY2RGBA);
         case GRAY16 ->
         {
            OpenCVTools.clampTo8BitUnsignedChar(imageToPack, imageToPack, 0.0, 255.0);
            OpenCVTools.convertGrayToRGBA(imageToPack, imageToPack);
         }
      }

      lastImagePixelFormat = PixelFormat.RGBA8;
   }

   public void decodeMessageToRGBA(ImageMessage messageToDecode, GpuMat imageToPack)
   {
      decodeMessage(messageToDecode, imageToPack);

      switch (lastImagePixelFormat)
      {
         case YUVI420 -> opencv_cudaimgproc.cvtColor(imageToPack, imageToPack, opencv_imgproc.COLOR_YUV2RGBA_I420);
         case BGR8 -> opencv_cudaimgproc.cvtColor(imageToPack, imageToPack, opencv_imgproc.COLOR_BGR2RGBA);
         case BGRA8 -> opencv_cudaimgproc.cvtColor(imageToPack, imageToPack, opencv_imgproc.COLOR_BGRA2RGBA);
         case RGB8 -> opencv_cudaimgproc.cvtColor(imageToPack, imageToPack, opencv_imgproc.COLOR_RGB2RGBA);
         case GRAY8 -> opencv_cudaimgproc.cvtColor(imageToPack, imageToPack, opencv_imgproc.COLOR_GRAY2RGBA);
         case GRAY16 ->
         {
            OpenCVTools.cudaClampTo8BitUnsignedChar(imageToPack, imageToPack, 0.0, 255.0);
            opencv_cudaimgproc.cvtColor(imageToPack, imageToPack, opencv_imgproc.COLOR_GRAY2RGBA);
         }
      }

      lastImagePixelFormat = PixelFormat.RGBA8;
   }

   public PixelFormat getDecodedImagePixelFormat()
   {
      return lastImagePixelFormat;
   }

   public void destroy()
   {
      if (cudaCompressionTools != null)
         cudaCompressionTools.destroy();
      if (cudaJpegDecoder != null)
         cudaJpegDecoder.destroy();

      messageDataExtractor.destroy();
   }

   private void resizeToMessageDimensions(ImageMessage imageMessage, Mat imageToResize)
   {
      if (imageToResize.cols() != imageMessage.getImageWidth())
         imageToResize.cols(imageMessage.getImageWidth());
      if (imageToResize.rows() != imageMessage.getImageHeight())
         imageToResize.rows(imageMessage.getImageHeight());
   }

   private void resizeToMessageDimensions(ImageMessage imageMessage, GpuMat imageToResize)
   {
      if (imageToResize.cols() != imageMessage.getImageWidth())
         imageToResize.cols(imageMessage.getImageWidth());
      if (imageToResize.rows() != imageMessage.getImageHeight())
         imageToResize.rows(imageMessage.getImageHeight());
   }
}
