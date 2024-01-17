package us.ihmc.perception.opencv;

import org.bytedeco.opencv.opencv_objdetect.DetectorParameters;

/**
 * Needed to tune the parameters while copying them to the swap buffer used to
 * detect markers on another thread.
 */
public class OpenCVArUcoMarkerDetectionTools
{
   public static void copy(DetectorParameters from, DetectorParameters to)
   {
      to.adaptiveThreshWinSizeMin(from.adaptiveThreshWinSizeMin());
      to.adaptiveThreshWinSizeMax(from.adaptiveThreshWinSizeMax());
      to.adaptiveThreshWinSizeStep(from.adaptiveThreshWinSizeStep());
      to.adaptiveThreshConstant(from.adaptiveThreshConstant());
      to.minMarkerPerimeterRate(from.minMarkerPerimeterRate());
      to.maxMarkerPerimeterRate(from.maxMarkerPerimeterRate());
      to.polygonalApproxAccuracyRate(from.polygonalApproxAccuracyRate());
      to.minCornerDistanceRate(from.minCornerDistanceRate());
      to.minDistanceToBorder(from.minDistanceToBorder());
      to.minMarkerDistanceRate(from.minMarkerDistanceRate());
      to.cornerRefinementMethod(from.cornerRefinementMethod());
      to.cornerRefinementWinSize(from.cornerRefinementWinSize());
      to.cornerRefinementMaxIterations(from.cornerRefinementMaxIterations());
      to.cornerRefinementMinAccuracy(from.cornerRefinementMinAccuracy());
      to.markerBorderBits(from.markerBorderBits());
      to.perspectiveRemovePixelPerCell(from.perspectiveRemovePixelPerCell());
      to.perspectiveRemoveIgnoredMarginPerCell(from.perspectiveRemoveIgnoredMarginPerCell());
      to.maxErroneousBitsInBorderRate(from.maxErroneousBitsInBorderRate());
      to.minOtsuStdDev(from.minOtsuStdDev());
      to.errorCorrectionRate(from.errorCorrectionRate());
      to.aprilTagQuadDecimate(from.aprilTagQuadDecimate());
      to.aprilTagQuadSigma(from.aprilTagQuadSigma());
      to.aprilTagMinClusterPixels(from.aprilTagMinClusterPixels());
      to.aprilTagMaxNmaxima(from.aprilTagMaxNmaxima());
      to.aprilTagCriticalRad(from.aprilTagCriticalRad());
      to.aprilTagMaxLineFitMse(from.aprilTagMaxLineFitMse());
      to.aprilTagMinWhiteBlackDiff(from.aprilTagMinWhiteBlackDiff());
      to.aprilTagDeglitch(from.aprilTagDeglitch());
      to.detectInvertedMarker(from.detectInvertedMarker());
      to.useAruco3Detection(from.useAruco3Detection());
      to.minSideLengthCanonicalImg(from.minSideLengthCanonicalImg());
      to.minMarkerLengthRatioOriginalImg(from.minMarkerLengthRatioOriginalImg());
   }
}
