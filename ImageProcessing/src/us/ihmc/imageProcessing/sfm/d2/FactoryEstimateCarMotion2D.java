package us.ihmc.imageProcessing.sfm.d2;

import georegression.struct.se.Se2_F64;
import us.ihmc.imageProcessing.sfm.d2.wrappers.MonoOverhead_to_CarMotion2D;
import us.ihmc.imageProcessing.sfm.d2.wrappers.Mono_to_CarMotion2D;
import us.ihmc.imageProcessing.sfm.d2.wrappers.StereoVO_to_CarMotion2D;
import boofcv.abst.feature.detect.interest.ConfigGeneralDetector;
import boofcv.abst.feature.disparity.StereoDisparitySparse;
import boofcv.abst.feature.tracker.PointTracker;
import boofcv.abst.feature.tracker.PointTrackerTwoPass;
import boofcv.abst.sfm.d2.ImageMotion2D;
import boofcv.abst.sfm.d3.MonocularPlaneVisualOdometry;
import boofcv.abst.sfm.d3.MonocularPlaneVisualOdometryScaleInput;
import boofcv.abst.sfm.d3.StereoVisualOdometry;
import boofcv.abst.sfm.d3.StereoVisualOdometryScaleInput;
import boofcv.alg.tracker.klt.PkltConfig;
import boofcv.factory.feature.disparity.FactoryStereoDisparity;
import boofcv.factory.feature.tracker.FactoryPointTracker;
import boofcv.factory.feature.tracker.FactoryPointTrackerTwoPass;
import boofcv.factory.sfm.FactoryMotion2D;
import boofcv.factory.sfm.FactoryVisualOdometry;
import boofcv.struct.image.ImageFloat32;
import boofcv.struct.image.ImageType;

/**
 * @author Peter Abeles
 */
public class FactoryEstimateCarMotion2D
{
   public static EstimateCarMotion2D monoOverhead() {

      PkltConfig config = new PkltConfig();
      config.pyramidScaling = new int[]{1,2,4,8};
      config.templateRadius = 3;
      ConfigGeneralDetector configDetector = new ConfigGeneralDetector(600,3,1);

      PointTracker<ImageFloat32> tracker = FactoryPointTracker.klt(config, configDetector,ImageFloat32.class, ImageFloat32.class);

      int ransacIterations = 300;
      double inlierGroundTol = 0.2;
      int thresholdRetire = 2;
      int absoluteMinimumTracks = 30;
      double respawnTrackFraction = 0.4;
      double respawnCoverageFraction = 0.6;

      ImageMotion2D<ImageFloat32,Se2_F64> motion2D = FactoryMotion2D.createMotion2D(
            ransacIterations, inlierGroundTol * inlierGroundTol, thresholdRetire,
            absoluteMinimumTracks, respawnTrackFraction, respawnCoverageFraction, false, tracker, new Se2_F64());

      return new MonoOverhead_to_CarMotion2D(motion2D);
   }

   public static EstimateCarMotion2D monoPlaneInfinity(double scale) {

      // specify how the image features are going to be tracked
      PkltConfig configKlt = new PkltConfig();
      configKlt.pyramidScaling = new int[]{1,2,4,8};
      configKlt.templateRadius = 5;

      PointTrackerTwoPass<ImageFloat32> tracker = FactoryPointTrackerTwoPass.klt(configKlt, new ConfigGeneralDetector(-1, 3, 150),
            ImageFloat32.class, ImageFloat32.class);

      // declares the algorithm
      MonocularPlaneVisualOdometry<ImageFloat32> vo = FactoryVisualOdometry.monoPlaneInfinity(100, 2, 1.5, 200, tracker,
            ImageType.single(ImageFloat32.class));

      if( scale != 1.0 ) {
         vo = new MonocularPlaneVisualOdometryScaleInput<ImageFloat32>(vo,scale);
      }

      return new Mono_to_CarMotion2D(vo);
   }

   public static EstimateCarMotion2D stereo01( double scale ) {
      // specify how the image features are going to be tracked
      PkltConfig configKlt = new PkltConfig();
      configKlt.pyramidScaling = new int[]{1,2,4,8};
      configKlt.templateRadius = 5;

      PointTrackerTwoPass<ImageFloat32> tracker = FactoryPointTrackerTwoPass.klt(configKlt, new ConfigGeneralDetector(-1, 3, 150),
            ImageFloat32.class, ImageFloat32.class);

      // computes the depth of each point
      StereoDisparitySparse<ImageFloat32> disparity = FactoryStereoDisparity.regionSparseWta(0, 10, 3, 3, 20, 0.15, true, ImageFloat32.class);

      // declares the algorithm
      StereoVisualOdometry<ImageFloat32> vo = FactoryVisualOdometry.stereoDepth(1.5, 200, 2, 200, 50, false, disparity, tracker, ImageFloat32.class);

      if( scale != 1.0 ) {
         vo = new StereoVisualOdometryScaleInput<ImageFloat32>(vo,scale);
      }

      return new StereoVO_to_CarMotion2D(vo);
   }
}
