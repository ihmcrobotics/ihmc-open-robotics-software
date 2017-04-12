package us.ihmc.ihmcPerception;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;

import javax.imageio.ImageIO;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class OpenCVTools
{
   public static final String OPEN_CV_LIBRARY_NAME = "opencv_java310";
   
   public static Mat convertBufferedImageToMat(BufferedImage image)
   {
      Mat imageMat;
      byte[] pixel;
      switch (image.getType())
      {
      case BufferedImage.TYPE_3BYTE_BGR:
         imageMat = new Mat(image.getHeight(), image.getWidth(), CvType.CV_8UC3);
         pixel = ((DataBufferByte) (image.getRaster().getDataBuffer())).getData();
         break;
      case BufferedImage.TYPE_4BYTE_ABGR:
      case BufferedImage.TYPE_INT_ARGB:
         //need to double check endianess
         imageMat = new Mat(image.getHeight(), image.getWidth(), CvType.CV_8UC4);
         pixel = new byte[image.getHeight() * image.getWidth() * 4];
         ByteBuffer pixelBuf = ByteBuffer.wrap(pixel);
         int[] intBuf = image.getRGB(0, 0, image.getWidth(), image.getHeight(), null, 0, image.getWidth());
         pixelBuf.asIntBuffer().put(intBuf);
         break;
      default:
         throw new RuntimeException("unknown type " + image.getType());
      }
      imageMat.put(0, 0, pixel);
      return imageMat;
   }
   
   public static BufferedImage convertToCompressableBufferedImage(BufferedImage openCVEncodedImage)
   {
      BufferedImage convertedBufferedImage = new BufferedImage(openCVEncodedImage.getWidth(),
                                                               openCVEncodedImage.getHeight(),
                                                               BufferedImage.TYPE_INT_RGB);
      convertedBufferedImage.getGraphics().drawImage(openCVEncodedImage, 0, 0, null);

      return convertedBufferedImage;
   }
   
   public static BufferedImage convertMatToBufferedImage(Mat image)
   {
      MatOfByte matOfByte = new MatOfByte();
      Imgcodecs.imencode(".bmp", image, matOfByte);
      BufferedImage bufferedImage = null;
      try
      {
         bufferedImage = ImageIO.read(new ByteArrayInputStream(matOfByte.toArray()));
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      
      return bufferedImage;
   }
   
   public static void resizeImage(Mat image, double scaleFactor)
   {
      Imgproc.resize(image, image, new Size((int) (image.width() * scaleFactor), (int) (image.height() * scaleFactor)));
   }
}
