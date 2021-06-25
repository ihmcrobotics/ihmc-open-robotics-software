package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class AdaptiveSwingTimingTools
{
   public static double calculateSwingTime(double idealStepLength,
                                           double maxSwingReach,
                                           double maxStepHeight,
                                           double minSwingTime,
                                           double maxSwingTime,
                                           Point3DReadOnly startPosition,
                                           Point3DReadOnly endPosition)
   {
      double idealStrideLength = 2.0 * idealStepLength;
      double maxStrideDistance = EuclidCoreTools.norm(maxSwingReach, 2.0 * maxStepHeight);

      double strideLength = startPosition.distance(endPosition);
      double alpha = MathTools.clamp((strideLength - idealStrideLength) / (maxStrideDistance - idealStrideLength), 0.0, 1.0);

      return minSwingTime + alpha * (maxSwingTime - minSwingTime);
   }

   public static double calculateTransferTime(double idealStepLength,
                                              double idealStepWidth,
                                              double maxStepLength,
                                              double maxStepHeight,
                                              double minSwingTime,
                                              double maxSwingTime,
                                              Point3DReadOnly transferFromFootPosition,
                                              Point3DReadOnly transferToFootPosition)
   {
      double maxDistance = EuclidCoreTools.norm(maxStepLength, idealStepWidth, maxStepHeight);
      double idealDistance = EuclidCoreTools.norm(idealStepLength, idealStepWidth);

      double alpha = MathTools.clamp((transferFromFootPosition.distance(transferToFootPosition) - idealDistance) / (maxDistance - idealDistance), 0.0, 1.0);
      return InterpolationTools.linearInterpolate(minSwingTime, maxSwingTime, alpha);
   }
}
