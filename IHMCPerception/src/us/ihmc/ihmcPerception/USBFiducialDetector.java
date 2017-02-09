package us.ihmc.ihmcPerception;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.gui.fiducial.VisualizeFiducial;
import boofcv.gui.image.ImagePanel;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.ImageType;
import georegression.struct.se.Se3_F64;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;

public class USBFiducialDetector
{
   static
   {
      NativeLibraryLoader.loadLibrary("org.opencv", OpenCVTools.OPEN_CV_LIBRARY_NAME);
   }

   public static void main(String[] args) throws IOException
   {
      FiducialDetector<GrayF32> detector = FactoryFiducial
            .squareBinary(new ConfigFiducialBinary(0.1), ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10), GrayF32.class);

      VideoCapture videoCapture = new VideoCapture(0);
      IntrinsicParameters intrinsicParameters = new IntrinsicParameters(476, 476, 0, 640, 360, 1280, 720);
      detector.setIntrinsic(intrinsicParameters);

      ImagePanel imagePanel = null;

      Mat image = new Mat();
      MatOfByte matOfByte = new MatOfByte();
      GrayF32 formattedImage;

      while (true)
      {
         try
         {
            videoCapture.read(image);

            Imgcodecs.imencode(".bmp", image, matOfByte);
            BufferedImage bufferedImage = ImageIO.read(new ByteArrayInputStream(matOfByte.toArray()));
            formattedImage = ConvertBufferedImage.convertFrom(bufferedImage, true, ImageType.single(GrayF32.class));

            detector.detect(formattedImage);

            Graphics2D g2 = bufferedImage.createGraphics();
            Se3_F64 targetToSensor = new Se3_F64();

            for (int i = 0; i < detector.totalFound(); i++)
            {
               detector.getFiducialToCamera(i, targetToSensor);

               VisualizeFiducial.drawCube(targetToSensor, intrinsicParameters, detector.getWidth(i), 3, g2);
               VisualizeFiducial.drawLabelCenter(targetToSensor, intrinsicParameters, "" + detector.getId(i), g2);


               System.out.println("target pos: (" + targetToSensor.getX() + ", " + targetToSensor.getY() + ", " + targetToSensor.getZ() + ")");
            }

            if (imagePanel == null)
            {
               imagePanel = ShowImages.showWindow(bufferedImage, "video");
            }
            else
            {
               imagePanel.setBufferedImageSafe(bufferedImage);
            }
         }
         catch (Exception e)
         {

         }
      }
   }

}
