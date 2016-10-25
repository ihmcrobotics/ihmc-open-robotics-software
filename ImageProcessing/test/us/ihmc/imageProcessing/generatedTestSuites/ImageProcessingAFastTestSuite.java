package us.ihmc.imageProcessing.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite.ContinuousIntegrationSuiteCategory;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(ContinuousIntegrationSuite.class)
@ContinuousIntegrationSuiteCategory(IntegrationCategory.FAST)
@SuiteClasses
({
   us.ihmc.imageProcessing.segmentation.FitNoisyGaussian2DTest.class,
   us.ihmc.imageProcessing.segmentation.Gaussian2D_F64Test.class,
   us.ihmc.imageProcessing.segmentation.LabeledPixelCodecTest.class,
   us.ihmc.imageProcessing.sfm.EstimateGroundPlaneFromFeaturesTest.class
})

public class ImageProcessingAFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
