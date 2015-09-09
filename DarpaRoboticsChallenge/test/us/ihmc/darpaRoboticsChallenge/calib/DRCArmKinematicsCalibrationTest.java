package us.ihmc.darpaRoboticsChallenge.calib;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;
import georegression.struct.se.Se3_F64;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.awt.image.BufferedImage;

import org.junit.Test;

import boofcv.io.image.UtilImageIO;
import boofcv.struct.calib.IntrinsicParameters;

/**
 * @author Peter Abeles
 */
public class DRCArmKinematicsCalibrationTest
{

	@DeployableTestMethod(duration = 0.7)
	@Test(timeout = 30000)
   public void estimateCameraPose()
   {
      BufferedImage image = UtilImageIO.loadImage("../DarpaRoboticsChallenge/data/atlas_chessboard.jpg");

      if (image == null)
         fail("can't find the test image.");

      DRCArmKinematicsCalibration alg = new DRCArmKinematicsCalibration();

      // if param has not been set it should fail
      assertFalse(alg.estimateCameraPose(image));

      // just make up some parameters
      int w = image.getWidth();
      int h = image.getHeight();
      IntrinsicParameters param = new IntrinsicParameters(500, 500, 0, w / 2, h / 2, w, h, false, null);
      alg.setIntrinsic(param);

      assertTrue(alg.estimateCameraPose(image));

      Se3_F64 pose = alg.getTargetToOrigin();

      assertTrue(pose.T.x != 0);
      assertTrue(pose.T.y != 0);
      assertTrue(pose.T.z > 0.2);
   }
}
