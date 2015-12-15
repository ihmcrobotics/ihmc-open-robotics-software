package us.ihmc.imageProcessing.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.testing.TestPlanSuite;
import us.ihmc.tools.testing.TestPlanSuite.TestSuiteTarget;
import us.ihmc.tools.testing.TestPlanTarget;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(TestPlanSuite.class)
@TestSuiteTarget(TestPlanTarget.Fast)
@SuiteClasses
({
   us.ihmc.imageProcessing.segmentation.Gaussian2D_F64Test.class,
   us.ihmc.imageProcessing.segmentation.FitNoisyGaussian2DTest.class,
   us.ihmc.imageProcessing.segmentation.LabeledPixelCodecTest.class,
   us.ihmc.imageProcessing.segmentation.GaussianColorClassifierTest.class,
   us.ihmc.imageProcessing.segmentation.Gaussian3D_F64Test.class,
   us.ihmc.imageProcessing.sfm.EstimateGroundPlaneFromFeaturesTest.class,
   us.ihmc.imageProcessing.sfm.ImageToGroundMapTest.class
})

public class ImageProcessingAFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
