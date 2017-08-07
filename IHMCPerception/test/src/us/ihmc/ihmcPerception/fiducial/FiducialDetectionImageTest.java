package us.ihmc.ihmcPerception.fiducial;

import static org.junit.Assert.assertEquals;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.file.Paths;

import javax.imageio.ImageIO;

import org.junit.Test;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.gui.fiducial.VisualizeFiducial;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.ImageType;
import georegression.struct.se.Se3_F64;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.tools.io.resources.ResourceTools;
import us.ihmc.tools.thread.ThreadTools;

public class FiducialDetectionImageTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testFiducialDetected() throws IOException
   {
      FiducialDetector<GrayF32> detector = FactoryFiducial.squareBinary(new ConfigFiducialBinary(0.1), ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10),
                                                                        GrayF32.class);

      IntrinsicParameters intrinsicParameters = new IntrinsicParameters(476, 476, 0, 640, 360, 1280, 720);
      detector.setIntrinsic(intrinsicParameters);

      BufferedImage image = ImageIO.read(ResourceTools.openStreamRelative(getClass(), Paths.get("FiducialDetection1.png")));
      GrayF32 grayImage = ConvertBufferedImage.convertFrom(image, true, ImageType.single(GrayF32.class));

      detector.detect(grayImage);

      assertEquals("Fiducial not found", 1, detector.totalFound());

      Se3_F64 fiducialToCamera = new Se3_F64();
      detector.getFiducialToCamera(0, fiducialToCamera);

      PrintTools.info("Fiducial detected: " + fiducialToCamera.getTranslation());
      
      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
      {
         Graphics2D graphics = image.createGraphics();

         VisualizeFiducial.drawCube(fiducialToCamera, intrinsicParameters, detector.getWidth(0), 3, graphics);
         VisualizeFiducial.drawLabelCenter(fiducialToCamera, intrinsicParameters, "" + detector.getId(0), graphics);
         
         ShowImages.showWindow(image, "FiducialDetectionImageTest");
         
         ThreadTools.sleepForever();
      }
   }
}
