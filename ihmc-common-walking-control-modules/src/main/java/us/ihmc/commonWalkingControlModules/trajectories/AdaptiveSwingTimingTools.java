package us.ihmc.commonWalkingControlModules.trajectories;

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
                                           Point3DReadOnly startPosition, Point3DReadOnly endPosition)
   {
      double idealStrideLength = 2.0 * idealStepLength;
      double maxStrideDistance = EuclidCoreTools.norm(maxSwingReach, 2.0 * maxStepHeight);

      double strideLength = startPosition.distance(endPosition);
      double alpha = MathTools.clamp((strideLength - idealStrideLength) / (maxStrideDistance - idealStrideLength), 0.0, 1.0);

      return minSwingTime + alpha * (maxSwingTime - minSwingTime);
   }
}
