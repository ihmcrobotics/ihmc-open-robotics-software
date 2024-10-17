package us.ihmc.perception.fiducial;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.alg.distort.brown.LensDistortionBrown;
import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.gui.fiducial.VisualizeFiducial;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.ImageType;
import georegression.struct.se.Se3_F64;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.resources.ResourceTools;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.file.Paths;

import static org.junit.jupiter.api.Assertions.*;

public class FiducialDetectionImageTest
{
   @Test
   public void testFiducialDetected() throws IOException
   {
      FiducialDetector<GrayF32> detector = FactoryFiducial.squareBinary(new ConfigFiducialBinary(0.1), ConfigThreshold.local(ThresholdType.LOCAL_GAUSSIAN, 10),
                                                                        GrayF32.class);

      CameraPinholeBrown intrinsicParameters = new CameraPinholeBrown(476, 476, 0, 640, 360, 1280, 720);
      detector.setLensDistortion(new LensDistortionBrown(intrinsicParameters), intrinsicParameters.width, intrinsicParameters.height);

      BufferedImage image = ImageIO.read(ResourceTools.openStreamRelative(getClass(), Paths.get("FiducialDetection1.png")));
      GrayF32 grayImage = ConvertBufferedImage.convertFrom(image, true, ImageType.single(GrayF32.class));

      detector.detect(grayImage);

      assertEquals(1, detector.totalFound(), "Fiducial not found");

      Se3_F64 fiducialToCamera = new Se3_F64();
      detector.getFiducialToCamera(0, fiducialToCamera);

      LogTools.info("Fiducial detected: {}", fiducialToCamera.getTranslation());
      
      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
      {
         Graphics2D graphics = image.createGraphics();

         VisualizeFiducial.drawCube(fiducialToCamera, intrinsicParameters, detector.getWidth(0), 3, graphics);
         VisualizeFiducial.drawLabelCenter(fiducialToCamera, intrinsicParameters, "" + detector.getId(0), graphics);
         
         ShowImages.showWindow(image, "FiducialDetectionImageTest");
         
         ThreadTools.sleepSeconds(2.0);
      }
   }
}
