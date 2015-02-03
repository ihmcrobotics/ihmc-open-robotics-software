package us.ihmc.imageProcessing.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

//import us.ihmc.utilities.code.unitTesting.runner.JUnitTestSuiteRunner;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.imageProcessing.driving.DrcColorPixelDrivingClassifierTest.class,
   us.ihmc.imageProcessing.driving.OccupancyGridTest.class,
   us.ihmc.imageProcessing.segmentation.FitNoisyGaussian2DTest.class,
   us.ihmc.imageProcessing.segmentation.Gaussian2D_F64Test.class,
   us.ihmc.imageProcessing.segmentation.Gaussian3D_F64Test.class,
   us.ihmc.imageProcessing.segmentation.GaussianColorClassifierTest.class,
   us.ihmc.imageProcessing.segmentation.LabeledPixelCodecTest.class,
   us.ihmc.imageProcessing.sfm.EstimateGroundPlaneFromFeaturesTest.class,
   us.ihmc.imageProcessing.sfm.ImageToGroundMapTest.class
})

public class ImageProcessingDockerTestSuite
{
   public static void main(String[] args)
   {
      //new JUnitTestSuiteRunner(ImageProcessingDockerTestSuite.class);
   }
}

