package us.ihmc.commonWalkingControlModules.capturePoint;

import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Note: CMP stands for Centroidal Momentum Pivot
 *
 */
public class CapturePointTools
{
   private static final double EPSILON = 1.0e-15;

   /**
    * Compute the constant CMP locations and store them in constantCMPsToPack.
    *
    * @param constantCMPsToPack List that will be packed with the constant CMP locations
    * @param footstepList List containing the footsteps
    * @param firstFootstepIndex Integer describing the index of the first footstep to consider when
    *           laying out the CMP's
    * @param lastFootstepIndex Integer describing the index of the last footstep to consider when
    *           laying out the CMP's
    * @param startStanding If true, the first constant CMP will be between the 2 first footsteps,
    *           else it will at the first footstep.
    * @param endStanding If true, the last constant CMP will be between the 2 last footsteps, else
    *           it will at the last footstep.
    */
   public static void computeConstantCMPs(List<YoFramePoint> constantCMPsToPack, List<FramePoint3D> footstepList, int firstFootstepIndex, int lastFootstepIndex,
                                          boolean startStanding, boolean endStanding)
   {
      if (startStanding)
      {
         // Start with the first constant CMP located between the feet.
         YoFramePoint firstConstantCMPPlanned = constantCMPsToPack.get(firstFootstepIndex);
         FramePoint3D firstFootstepToConsider = footstepList.get(firstFootstepIndex);
         FramePoint3D secondFootstepToConsider = footstepList.get(firstFootstepIndex + 1);
         putConstantCMPBetweenFeet(firstConstantCMPPlanned, firstFootstepToConsider, secondFootstepToConsider);
         firstFootstepIndex++;
      }

      if (endStanding)
      {
         // End with the last constant CMP located between the feet.
         YoFramePoint lastConstantCMPPlanned = constantCMPsToPack.get(lastFootstepIndex);
         FramePoint3D lastFootstepToConsider = footstepList.get(lastFootstepIndex);
         FramePoint3D beforeLastFootstepToConsider = footstepList.get(lastFootstepIndex - 1);
         putConstantCMPBetweenFeet(lastConstantCMPPlanned, beforeLastFootstepToConsider, lastFootstepToConsider);
         lastFootstepIndex--;
      }

      computeConstantCMPsOnFeet(constantCMPsToPack, footstepList, firstFootstepIndex, lastFootstepIndex);
   }

   /**
    * Put the constant CMP's on the footsteps.
    *
    * @param constantCMPsToPack List that will be packed with the constant CMP locations
    * @param footstepList List containing the footsteps
    * @param firstFootstepIndex Integer describing the index of the first footstep to consider when
    *           laying out the CMP's
    * @param lastFootstepIndex Integer describing the index of the last footstep to consider when
    *           laying out the CMP's
    */
   public static void computeConstantCMPsOnFeet(List<YoFramePoint> constantCMPsToPack, List<FramePoint3D> footstepList, int firstFootstepIndex,
                                                int lastFootstepIndex)
   {
      for (int i = firstFootstepIndex; i <= lastFootstepIndex; i++)
      {
         YoFramePoint constantCMP = constantCMPsToPack.get(i);
         // Put the constant CMP at the footstep location
         constantCMP.setAndMatchFrame(footstepList.get(i));
      }
   }

   /**
    * Put the constant CMP in the middle of the two given footsteps.
    * 
    * @param constantCMPToPack YoFramePoint that will be packed with the constant CMP location
    * @param firstFootstep FramePoint holding the position of the first footstep
    * @param secondFootstep FramePoint holding the position of the second footstep
    */
   public static void putConstantCMPBetweenFeet(YoFramePoint constantCMPToPack, FramePoint3D firstFootstep, FramePoint3D secondFootstep)
   {
      constantCMPToPack.setAndMatchFrame(firstFootstep);
      constantCMPToPack.add(secondFootstep);
      constantCMPToPack.scale(0.5);
   }

   /**
    * Backward calculation of desired end of step capture point locations.
    * 
    * @param cornerPointsToPack
    * @param constantCMPs
    * @param skipFirstCornerPoint whether the first is to be skipped or not. When in double support,
    *           the first corner point is useless and usually skipped.
    * @param stepTime equals to the single plus double support durations
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    */
   public static void computeDesiredCornerPoints(List<? extends YoFramePoint> cornerPointsToPack, List<YoFramePoint> constantCMPs, boolean skipFirstCornerPoint,
                                                 double stepTime, double omega0)
   {
      double exponentialTerm = Math.exp(-omega0 * stepTime);
      YoFramePoint nextCornerPoint = constantCMPs.get(cornerPointsToPack.size());

      int firstCornerPointIndex = skipFirstCornerPoint ? 1 : 0;
      for (int i = cornerPointsToPack.size() - 1; i >= firstCornerPointIndex; i--)
      {
         YoFramePoint cornerPoint = cornerPointsToPack.get(i);
         YoFramePoint initialCMP = constantCMPs.get(i);

         cornerPoint.interpolate(initialCMP, nextCornerPoint, exponentialTerm);

         nextCornerPoint = cornerPoint;
      }

      if (skipFirstCornerPoint)
         cornerPointsToPack.get(0).setToNaN();
   }

   /**
    * Backward calculation of the ICP corner points as the method
    * {@link #computeDesiredCornerPoints(List, List, boolean, double, double)} but considering
    * transfer time and swing time on a per step basis.
    * <p>
    * This method is to be used when in double support, or transfer. The difference with single
    * support is the presence of an additional constant CMP for the trailing foot for which the
    * corner point does not need to be computed and also that offsets the transfer/swing time index.
    * 
    * @param cornerPointsToPack the ICP corner points computed by this method. Modified.
    * @param constantCMPs the constant CMPs already computed. Not modified.
    * @param swingTimes the swing time on a per step basis. Not modified.
    * @param transferTimes the transfer time on a per step basis. Not modified.
    * @param doubleSupportSplitFraction repartition around the ICP entry corner point of the double
    *           support.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    */
   public static void computeDesiredCornerPointsDoubleSupport(List<? extends YoFramePoint> cornerPointsToPack, List<YoFramePoint> constantCMPs,
                                                              List<YoDouble> swingTimes, List<YoDouble> transferTimes,
                                                              double doubleSupportSplitFraction, double omega0)
   {
      YoFramePoint nextCornerPoint = constantCMPs.get(cornerPointsToPack.size());

      for (int i = cornerPointsToPack.size() - 1; i >= 1; i--)
      {
         double stepTime = swingTimes.get(i - 1).getDoubleValue();
         stepTime += (1.0 - doubleSupportSplitFraction) * transferTimes.get(i - 1).getDoubleValue();
         stepTime += doubleSupportSplitFraction * transferTimes.get(i).getDoubleValue();

         double exponentialTerm = Math.exp(-omega0 * stepTime);

         YoFramePoint cornerPoint = cornerPointsToPack.get(i);
         YoFramePoint initialCMP = constantCMPs.get(i);

         if (Double.isNaN(stepTime))
         {
            cornerPointsToPack.get(i).setToNaN();
            nextCornerPoint = constantCMPs.get(i);
            continue;
         }

         cornerPoint.interpolate(initialCMP, nextCornerPoint, exponentialTerm);

         nextCornerPoint = cornerPoint;
      }

      cornerPointsToPack.get(0).setToNaN();
   }

   /**
    * Backward calculation of the ICP corner points as the method
    * {@link #computeDesiredCornerPoints(List, List, boolean, double, double)} but considering
    * transfer time and swing time on a per step basis.
    * <p>
    * This method is to be used when in double support, or transfer. The difference with single
    * support is the presence of an additional constant CMP for the trailing foot for which the
    * corner point does not need to be computed and also that offsets the transfer/swing time index.
    *
    * @param cornerPointsToPack the ICP corner points computed by this method. Modified.
    * @param constantCMPs the constant CMPs already computed. Not modified.
    * @param swingTimes the swing time on a per step basis. Not modified.
    * @param transferTimes the transfer time on a per step basis. Not modified.
    * @param doubleSupportSplitFractions repartition around the ICP entry corner point of the double
    *           support.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    */
   public static void computeDesiredCornerPointsDoubleSupport(List<? extends YoFramePoint> cornerPointsToPack, List<YoFramePoint> constantCMPs,
         List<YoDouble> swingTimes, List<YoDouble> transferTimes, List<YoDouble> doubleSupportSplitFractions, double omega0)
   {
      YoFramePoint nextCornerPoint = constantCMPs.get(cornerPointsToPack.size());

      for (int i = cornerPointsToPack.size() - 1; i >= 1; i--)
      {
         double doubleSupportSplitFraction = doubleSupportSplitFractions.get(i - 1).getDoubleValue();
         double nextDoubleSupportSplitFraction = doubleSupportSplitFractions.get(i).getDoubleValue();

         double stepTime = swingTimes.get(i - 1).getDoubleValue();
         double transferTime = transferTimes.get(i - 1).getDoubleValue();
         double nextTransferTime = transferTimes.get(i).getDoubleValue();
         stepTime += (1.0 - doubleSupportSplitFraction) * transferTime;
         stepTime += nextDoubleSupportSplitFraction * nextTransferTime;

         if (Double.isNaN(nextTransferTime))
         {
            cornerPointsToPack.get(i).setToNaN();
            nextCornerPoint = constantCMPs.get(i);
            continue;
         }

         double exponentialTerm = Math.exp(-omega0 * stepTime);

         YoFramePoint cornerPoint = cornerPointsToPack.get(i);
         YoFramePoint initialCMP = constantCMPs.get(i);

         cornerPoint.interpolate(initialCMP, nextCornerPoint, exponentialTerm);

         nextCornerPoint = cornerPoint;
      }

      cornerPointsToPack.get(0).setToNaN();
   }

   /**
    * Backward calculation of the ICP corner points as the method
    * {@link #computeDesiredCornerPoints(List, List, boolean, double, double)} but considering
    * transfer time and swing time on a per step basis.
    * <p>
    * This method is to be used when in single support, or swing.
    * 
    * @param cornerPointsToPack the ICP corner points computed by this method. Modified.
    * @param constantCMPs the constant CMPs already computed. Not modified.
    * @param swingTimes the swing time on a per step basis. Not modified.
    * @param transferTimes the transfer time on a per step basis. Not modified.
    * @param doubleSupportSplitFraction repartition around the ICP entry corner point of the double
    *           support.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    */
   public static void computeDesiredCornerPointsSingleSupport(List<? extends YoFramePoint> cornerPointsToPack, List<YoFramePoint> constantCMPs,
                                                              List<YoDouble> swingTimes, List<YoDouble> transferTimes,
                                                              double doubleSupportSplitFraction, double omega0)
   {
      YoFramePoint nextCornerPoint = constantCMPs.get(cornerPointsToPack.size());

      for (int i = cornerPointsToPack.size() - 1; i >= 0; i--)
      {
         double stepTime = swingTimes.get(i).getDoubleValue();
         stepTime += (1.0 - doubleSupportSplitFraction) * transferTimes.get(i).getDoubleValue();
         stepTime += doubleSupportSplitFraction * transferTimes.get(i + 1).getDoubleValue();

         YoFramePoint cornerPoint = cornerPointsToPack.get(i);
         YoFramePoint initialCMP = constantCMPs.get(i);

         if (Double.isNaN(stepTime))
         {
            nextCornerPoint = constantCMPs.get(i);
            cornerPoint.setToNaN();
            continue;
         }

         double exponentialTerm = Math.exp(-omega0 * stepTime);

         cornerPoint.interpolate(initialCMP, nextCornerPoint, exponentialTerm);

         nextCornerPoint = cornerPoint;
      }
   }

   /**
    * Backward calculation of the ICP corner points as the method
    * {@link #computeDesiredCornerPoints(List, List, boolean, double, double)} but considering
    * transfer time and swing time on a per step basis.
    * <p>
    * This method is to be used when in single support, or swing.
    *
    * @param cornerPointsToPack the ICP corner points computed by this method. Modified.
    * @param constantCMPs the constant CMPs already computed. Not modified.
    * @param swingTimes the swing time on a per step basis. Not modified.
    * @param transferTimes the transfer time on a per step basis. Not modified.
    * @param doubleSupportSplitFractions repartition around the ICP entry corner point of the double
    *           support.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    */
   public static void computeDesiredCornerPointsSingleSupport(List<? extends YoFramePoint> cornerPointsToPack, List<YoFramePoint> constantCMPs,
         List<YoDouble> swingTimes, List<YoDouble> transferTimes,
         List<YoDouble> doubleSupportSplitFractions, double omega0)
   {
      YoFramePoint nextCornerPoint = constantCMPs.get(cornerPointsToPack.size());

      for (int i = cornerPointsToPack.size() - 1; i >= 0; i--)
      {
         double doubleSupportSplitFraction = doubleSupportSplitFractions.get(i).getDoubleValue();
         double nextDoubleSupportSplitFraction = doubleSupportSplitFractions.get(i + 1).getDoubleValue();

         double transferTime = transferTimes.get(i).getDoubleValue();
         double nextTransferTime = transferTimes.get(i + 1).getDoubleValue();
         double stepTime = swingTimes.get(i).getDoubleValue();
         stepTime += (1.0 - doubleSupportSplitFraction) * transferTime;
         stepTime += nextDoubleSupportSplitFraction * nextTransferTime;

         YoFramePoint cornerPoint = cornerPointsToPack.get(i);
         YoFramePoint initialCMP = constantCMPs.get(i);

         if (Double.isNaN(stepTime))
         {
            nextCornerPoint = constantCMPs.get(i);
            cornerPoint.setToNaN();
            continue;
         }

         double exponentialTerm = Math.exp(-omega0 * stepTime);

         cornerPoint.interpolate(initialCMP, nextCornerPoint, exponentialTerm);

         nextCornerPoint = cornerPoint;
      }
   }

   /**
    * Backward calculation of the ICP corner points as the method
    * {@link #computeDesiredCornerPoints(List, List, double, double)} but considering two constant
    * CMPs per support: an entryCMP and an exitCMP.
    * 
    * @param entryCornerPointsToPack the ICP entry corner points computed by this method. Modified.
    * @param exitCornerPointsToPack the ICP exit corner points computed by this method. Modified.
    * @param entryCMPs the entry constant CMPs already computed. Not modified.
    * @param exitCMPs the exit constant CMPs already computed. Not modified.
    * @param stepTime the step duration assumed to be the same for each step.
    * @param timeInPercentSpentOnExitCMPs repartition of the time between the entry and exit CMPs.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    */
   public static void computeDesiredCornerPoints(List<? extends YoFramePoint> entryCornerPointsToPack, List<? extends YoFramePoint> exitCornerPointsToPack,
                                                 List<YoFramePoint> entryCMPs, List<YoFramePoint> exitCMPs, double stepTime,
                                                 double timeInPercentSpentOnExitCMPs, double omega0)
   {
      double timeSpentOnExitCMP = stepTime * timeInPercentSpentOnExitCMPs;
      double timeSpentOnEntryCMP = stepTime * (1.0 - timeInPercentSpentOnExitCMPs);
      double entryExponentialTerm = Math.exp(-omega0 * timeSpentOnEntryCMP);
      double exitExponentialTerm = Math.exp(-omega0 * timeSpentOnExitCMP);

      YoFramePoint nextEntryCornerPoint = entryCMPs.get(entryCornerPointsToPack.size());

      for (int i = exitCornerPointsToPack.size() - 1; i >= 0; i--)
      {
         YoFramePoint exitCornerPoint = exitCornerPointsToPack.get(i);
         YoFramePoint entryCornerPoint = entryCornerPointsToPack.get(i);
         YoFramePoint exitCMP = exitCMPs.get(i);
         YoFramePoint entryCMP = entryCMPs.get(i);

         exitCornerPoint.interpolate(exitCMP, nextEntryCornerPoint, exitExponentialTerm);
         entryCornerPoint.interpolate(entryCMP, exitCornerPoint, entryExponentialTerm);

         nextEntryCornerPoint = entryCornerPoint;
      }
   }

   /**
    * Backward calculation of the ICP corner points as the method
    * {@link #computeDesiredCornerPoints(List, List, boolean, double, double)} but considering two
    * constant CMPs per support: an entryCMP and an exitCMP, and considering transfer time and swing
    * time on a per step basis.
    * <p>
    * This method is to be used when in double support, or transfer. The difference with single
    * support is the presence of an additional pair of entry/exit CMPs for the trailing foot for
    * which the entry/exit corner points do not need to be computed and also that offsets the
    * transfer/swing time index.
    * 
    * @param entryCornerPointsToPack the ICP entry corner points computed by this method. Modified.
    * @param exitCornerPointsToPack the ICP exit corner points computed by this method. Modified.
    * @param entryCMPs the entry constant CMPs already computed. Not modified.
    * @param exitCMPs the exit constant CMPs already computed. Not modified.
    * @param swingTimes the swing time on a per step basis. Not modified.
    * @param transferTimes the transfer time on a per step basis. Not modified.
    * @param swingSplitFraction repartition of the time between the entry and exit CMPs during
    *           swing.
    * @param transferSplitFraction repartition of the time between the exit and entry CMPs during
    *           transfer.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    */
   public static void computeDesiredCornerPointsDoubleSupport(List<? extends YoFramePoint> entryCornerPointsToPack,
                                                              List<? extends YoFramePoint> exitCornerPointsToPack, List<YoFramePoint> entryCMPs,
                                                              List<YoFramePoint> exitCMPs, List<YoDouble> swingTimes,
                                                              List<YoDouble> transferTimes, double swingSplitFraction, double transferSplitFraction,
                                                              double omega0)
   {
      YoFramePoint nextEntryCornerPoint = entryCMPs.get(entryCornerPointsToPack.size());

      for (int i = exitCornerPointsToPack.size() - 1; i >= 1; i--)
      {
         double upcomingSwingTime = swingTimes.get(i - 1).getDoubleValue();
         double transferTime = transferTimes.get(i - 1).getDoubleValue();
         double nextTransferTime = transferTimes.get(i).getDoubleValue();

         double timeSpentOnEntryCMP = upcomingSwingTime * swingSplitFraction + transferTime * (1.0 - transferSplitFraction);
         double timeSpentOnExitCMP = upcomingSwingTime * (1.0 - swingSplitFraction) + nextTransferTime * transferSplitFraction;
         double entryExponentialTerm = Math.exp(-omega0 * timeSpentOnEntryCMP);
         double exitExponentialTerm = Math.exp(-omega0 * timeSpentOnExitCMP);

         YoFramePoint exitCornerPoint = exitCornerPointsToPack.get(i);
         YoFramePoint entryCornerPoint = entryCornerPointsToPack.get(i);
         YoFramePoint exitCMP = exitCMPs.get(i);
         YoFramePoint entryCMP = entryCMPs.get(i);

         if (Double.isNaN(nextTransferTime))
         {
            nextEntryCornerPoint = entryCMPs.get(i);
            exitCornerPoint.setToNaN();
            entryCornerPoint.setToNaN();
            continue;
         }

         exitCornerPoint.interpolate(exitCMP, nextEntryCornerPoint, exitExponentialTerm);
         entryCornerPoint.interpolate(entryCMP, exitCornerPoint, entryExponentialTerm);

         nextEntryCornerPoint = entryCornerPoint;
      }

      exitCornerPointsToPack.get(0).setToNaN();
      entryCornerPointsToPack.get(0).setToNaN();
   }

   /**
    * Backward calculation of the ICP corner points as the method
    * {@link #computeDesiredCornerPoints(List, List, boolean, double, double)} but considering two
    * constant CMPs per support: an entryCMP and an exitCMP, and considering transfer time and swing
    * time on a per step basis.
    * <p>
    * This method is to be used when in double support, or transfer. The difference with single
    * support is the presence of an additional pair of entry/exit CMPs for the trailing foot for
    * which the entry/exit corner points do not need to be computed and also that offsets the
    * transfer/swing time index.
    *
    * @param entryCornerPointsToPack the ICP entry corner points computed by this method. Modified.
    * @param exitCornerPointsToPack the ICP exit corner points computed by this method. Modified.
    * @param entryCMPs the entry constant CMPs already computed. Not modified.
    * @param exitCMPs the exit constant CMPs already computed. Not modified.
    * @param swingTimes the swing time on a per step basis. Not modified.
    * @param transferTimes the transfer time on a per step basis. Not modified.
    * @param swingSplitFractions repartition of the time between the entry and exit CMPs during
    *           swing on a per step basis. Not modified.
    * @param transferSplitFractions repartition of the time between the exit and entry CMPs during
    *           transfer on a per step basis. Not modified.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    */
   public static void computeDesiredCornerPointsDoubleSupport(List<? extends YoFramePoint> entryCornerPointsToPack,
         List<? extends YoFramePoint> exitCornerPointsToPack, List<YoFramePoint> entryCMPs, List<YoFramePoint> exitCMPs,
         List<YoDouble> swingTimes, List<YoDouble> transferTimes, List<YoDouble> swingSplitFractions,
         List<YoDouble> transferSplitFractions, double omega0)
   {
      YoFramePoint nextEntryCornerPoint = entryCMPs.get(entryCornerPointsToPack.size());

      for (int i = exitCornerPointsToPack.size() - 1; i >= 1; i--)
      {
         double swingSplitFraction = swingSplitFractions.get(i - 1).getDoubleValue();
         double upcomingTransferSplitFraction = transferSplitFractions.get(i - 1).getDoubleValue();
         double nextTransferSplitFraction = transferSplitFractions.get(i).getDoubleValue();

         double upcomingSwingTime = swingTimes.get(i - 1).getDoubleValue();
         double transferTime = transferTimes.get(i - 1).getDoubleValue();
         double nextTransferTime = transferTimes.get(i).getDoubleValue();

         if (Double.isNaN(nextTransferTime) || Double.isNaN(nextTransferSplitFraction))
         {
            exitCornerPointsToPack.get(i).setToNaN();
            entryCornerPointsToPack.get(i).setToNaN();
            nextEntryCornerPoint = entryCMPs.get(i);
            continue;
         }

         double timeSpentOnEntryCMP = upcomingSwingTime * swingSplitFraction + transferTime * (1.0 - upcomingTransferSplitFraction);
         double timeSpentOnExitCMP = upcomingSwingTime * (1.0 - swingSplitFraction) + nextTransferTime * nextTransferSplitFraction;
         double entryExponentialTerm = Math.exp(-omega0 * timeSpentOnEntryCMP);
         double exitExponentialTerm = Math.exp(-omega0 * timeSpentOnExitCMP);

         YoFramePoint exitCornerPoint = exitCornerPointsToPack.get(i);
         YoFramePoint entryCornerPoint = entryCornerPointsToPack.get(i);
         YoFramePoint exitCMP = exitCMPs.get(i);
         YoFramePoint entryCMP = entryCMPs.get(i);

         exitCornerPoint.interpolate(exitCMP, nextEntryCornerPoint, exitExponentialTerm);
         entryCornerPoint.interpolate(entryCMP, exitCornerPoint, entryExponentialTerm);

         nextEntryCornerPoint = entryCornerPoint;
      }

      exitCornerPointsToPack.get(0).setToNaN();
      entryCornerPointsToPack.get(0).setToNaN();
   }

   /**
    * Backward calculation of the ICP corner points as the method
    * {@link #computeDesiredCornerPoints(List, List, boolean, double, double)} but considering two
    * constant CMPs per support: an entryCMP and an exitCMP, and considering transfer time and swing
    * time on a per step basis.
    * <p>
    * This method is to be used when in single support, or swing.
    * 
    * @param entryCornerPointsToPack the ICP entry corner points computed by this method. Modified.
    * @param exitCornerPointsToPack the ICP exit corner points computed by this method. Modified.
    * @param entryCMPs the entry constant CMPs already computed. Not modified.
    * @param exitCMPs the exit constant CMPs already computed. Not modified.
    * @param swingTimes the swing time on a per step basis. Not modified.
    * @param transferTimes the transfer time on a per step basis. Not modified.
    * @param swingSplitFraction repartition of the time between the entry and exit CMPs during
    *           swing.
    * @param transferSplitFraction repartition of the time between the exit and entry CMPs during
    *           transfer.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    */
   public static void computeDesiredCornerPointsSingleSupport(List<? extends YoFramePoint> entryCornerPointsToPack,
                                                              List<? extends YoFramePoint> exitCornerPointsToPack, List<YoFramePoint> entryCMPs,
                                                              List<YoFramePoint> exitCMPs, List<YoDouble> swingTimes,
                                                              List<YoDouble> transferTimes, double swingSplitFraction, double transferSplitFraction,
                                                              double omega0)
   {
      YoFramePoint nextEntryCornerPoint = entryCMPs.get(entryCornerPointsToPack.size());

      for (int i = exitCornerPointsToPack.size() - 1; i >= 0; i--)
      {
         double swingTime = swingTimes.get(i).getDoubleValue();
         double previousTransferTime = transferTimes.get(i).getDoubleValue();
         double nextTransferTime = transferTimes.get(i + 1).getDoubleValue();

         YoFramePoint exitCornerPoint = exitCornerPointsToPack.get(i);
         YoFramePoint entryCornerPoint = entryCornerPointsToPack.get(i);
         YoFramePoint exitCMP = exitCMPs.get(i);
         YoFramePoint entryCMP = entryCMPs.get(i);

         if (Double.isNaN(swingTime))
         {
            nextEntryCornerPoint = entryCMP;
            exitCornerPoint.setToNaN();
            entryCornerPoint.setToNaN();
            continue;
         }

         double timeSpentOnEntryCMP = swingTime * swingSplitFraction + previousTransferTime * (1.0 - transferSplitFraction);
         double timeSpentOnExitCMP = swingTime * (1.0 - swingSplitFraction) + nextTransferTime * transferSplitFraction;
         double entryExponentialTerm = Math.exp(-omega0 * timeSpentOnEntryCMP);
         double exitExponentialTerm = Math.exp(-omega0 * timeSpentOnExitCMP);

         exitCornerPoint.interpolate(exitCMP, nextEntryCornerPoint, exitExponentialTerm);
         entryCornerPoint.interpolate(entryCMP, exitCornerPoint, entryExponentialTerm);

         nextEntryCornerPoint = entryCornerPoint;
      }
   }

   /**
    * Backward calculation of the ICP corner points as the method
    * {@link #computeDesiredCornerPoints(List, List, boolean, double, double)} but considering two
    * constant CMPs per support: an entryCMP and an exitCMP, and considering transfer time and swing
    * time on a per step basis.
    * <p>
    * This method is to be used when in single support, or swing.
    *
    * @param entryCornerPointsToPack the ICP entry corner points computed by this method. Modified.
    * @param exitCornerPointsToPack the ICP exit corner points computed by this method. Modified.
    * @param entryCMPs the entry constant CMPs already computed. Not modified.
    * @param exitCMPs the exit constant CMPs already computed. Not modified.
    * @param swingTimes the swing time on a per step basis. Not modified.
    * @param transferTimes the transfer time on a per step basis. Not modified.
    * @param swingSplitFractions repartition of the time between the entry and exit CMPs during
    *           swing on a per step basis. Not modified.
    * @param transferSplitFractions repartition of the time between the exit and entry CMPs during
    *           transfer on a per step basis. Not modified.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    */
   public static void computeDesiredCornerPointsSingleSupport(List<? extends YoFramePoint> entryCornerPointsToPack,
         List<? extends YoFramePoint> exitCornerPointsToPack, List<YoFramePoint> entryCMPs, List<YoFramePoint> exitCMPs, List<YoDouble> swingTimes,
         List<YoDouble> transferTimes, List<YoDouble> swingSplitFractions, List<YoDouble> transferSplitFractions, double omega0)
   {
      YoFramePoint nextEntryCornerPoint = entryCMPs.get(entryCornerPointsToPack.size());

      for (int i = exitCornerPointsToPack.size() - 1; i >= 0; i--)
      {
         double swingSplitFraction = swingSplitFractions.get(i).getDoubleValue();
         double previousTransferSplitFraction = transferSplitFractions.get(i).getDoubleValue();
         double nextTransferSplitFraction = transferSplitFractions.get(i + 1).getDoubleValue();

         double swingTime = swingTimes.get(i).getDoubleValue();
         double previousTransferTime = transferTimes.get(i).getDoubleValue();
         double nextTransferTime = transferTimes.get(i + 1).getDoubleValue();

         YoFramePoint exitCornerPoint = exitCornerPointsToPack.get(i);
         YoFramePoint entryCornerPoint = entryCornerPointsToPack.get(i);
         YoFramePoint exitCMP = exitCMPs.get(i);
         YoFramePoint entryCMP = entryCMPs.get(i);

         if (Double.isNaN(swingTime))
         {
            nextEntryCornerPoint = entryCMP;
            exitCornerPoint.setToNaN();
            entryCornerPoint.setToNaN();
            continue;
         }

         double timeSpentOnEntryCMP = swingTime * swingSplitFraction + previousTransferTime * (1.0 - previousTransferSplitFraction);
         double timeSpentOnExitCMP = swingTime * (1.0 - swingSplitFraction) + nextTransferTime * nextTransferSplitFraction;
         double entryExponentialTerm = Math.exp(-omega0 * timeSpentOnEntryCMP);
         double exitExponentialTerm = Math.exp(-omega0 * timeSpentOnExitCMP);

         exitCornerPoint.interpolate(exitCMP, nextEntryCornerPoint, exitExponentialTerm);
         entryCornerPoint.interpolate(entryCMP, exitCornerPoint, entryExponentialTerm);

         nextEntryCornerPoint = entryCornerPoint;
      }
   }

   /**
    * Given a desired capturePoint location and an initial position of the capture point, compute
    * the constant CMP that will drive the capture point from the initial position to the final
    * position.
    * 
    * @param cmpToPack
    * @param finalDesiredCapturePoint
    * @param initialCapturePoint
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param stepTime
    */
   public static void computeConstantCMPFromInitialAndFinalCapturePointLocations(YoFramePoint cmpToPack, YoFramePoint finalDesiredCapturePoint,
                                                                                 YoFramePoint initialCapturePoint, double omega0, double stepTime)
   {
      double exponentialTerm = Math.exp(-omega0 * stepTime);
      cmpToPack.scaleSub(exponentialTerm, finalDesiredCapturePoint, initialCapturePoint);
      cmpToPack.scale(1.0 / (exponentialTerm - 1.0));
   }

   /**
    * Compute the desired capture point position at a given time. x<sup>ICP<sub>des</sub></sup> =
    * (e<sup>&omega;0 t</sup>) x<sup>ICP<sub>0</sub></sup> + (1-e<sup>&omega;0
    * t</sup>)x<sup>CMP<sub>0</sub></sup>
    * 
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointToPack
    */
   public static void computeDesiredCapturePointPosition(double omega0, double time, YoFramePoint initialCapturePoint, YoFramePoint initialCMP,
                                                         YoFramePoint desiredCapturePointToPack)
   {
      if (initialCapturePoint.distance(initialCMP) > EPSILON)
         desiredCapturePointToPack.interpolate(initialCMP, initialCapturePoint, Math.exp(omega0 * time));
      else
         desiredCapturePointToPack.set(initialCapturePoint);
   }

   /**
    * Compute the desired capture point position at a given time. x<sup>ICP<sub>des</sub></sup> =
    * (e<sup>&omega;0 t</sup>) x<sup>ICP<sub>0</sub></sup> + (1-e<sup>&omega;0
    * t</sup>)x<sup>CMP<sub>0</sub></sup>
    * 
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointToPack
    */
   public static void computeDesiredCapturePointPosition(double omega0, double time, FramePoint3D initialCapturePoint, FramePoint3D initialCMP,
                                                         FramePoint3D desiredCapturePointToPack)
   {
      desiredCapturePointToPack.setToZero(initialCapturePoint.getReferenceFrame());

      if (initialCapturePoint.distance(initialCMP) > EPSILON)
         desiredCapturePointToPack.interpolate(initialCMP, initialCapturePoint, Math.exp(omega0 * time));
      else
         desiredCapturePointToPack.set(initialCapturePoint);
   }

   /**
    * Compute the desired capture point position at a given time. x<sup>ICP<sub>des</sub></sup> =
    * (e<sup>&omega;0 t</sup>) x<sup>ICP<sub>0</sub></sup> + (1-e<sup>&omega;0
    * t</sup>)x<sup>CMP<sub>0</sub></sup>
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointToPack
    */
   public static void computeDesiredCapturePointPosition(double omega0, double time, FramePoint3D initialCapturePoint, FramePoint3D initialCMP,
                                                         YoFramePoint desiredCapturePointToPack)
   {
      if (initialCapturePoint.distance(initialCMP) > EPSILON)
         desiredCapturePointToPack.interpolate(initialCMP, initialCapturePoint, Math.exp(omega0 * time));
      else
         desiredCapturePointToPack.set(initialCapturePoint);
   }

   /**
    * Compute the desired capture point velocity at a given time. ICPv_d = w * e^{w*t} * ICP0 - p0 *
    * w * e^{w*t}
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointVelocityToPack
    */
   public static void computeDesiredCapturePointVelocity(double omega0, double time, YoFramePoint initialCapturePoint, YoFramePoint initialCMP,
                                                         YoFrameVector desiredCapturePointVelocityToPack)
   {
      if (initialCapturePoint.distance(initialCMP) > EPSILON)
      {
         desiredCapturePointVelocityToPack.sub(initialCapturePoint, initialCMP);
         desiredCapturePointVelocityToPack.scale(omega0 * Math.exp(omega0 * time));
      }
      else
         desiredCapturePointVelocityToPack.setToZero();
   }

   /**
    * Compute the desired capture point velocity at a given time. ICPv_d = w * * e^{w*t} * ICP0 - p0
    * * w * e^{w*t}
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointVelocityToPack
    */
   public static void computeDesiredCapturePointVelocity(double omega0, double time, FramePoint3D initialCapturePoint, FramePoint3D initialCMP,
                                                         FrameVector3D desiredCapturePointVelocityToPack)
   {
      desiredCapturePointVelocityToPack.setToZero(initialCapturePoint.getReferenceFrame());

      if (initialCapturePoint.distance(initialCMP) > EPSILON)
      {
         desiredCapturePointVelocityToPack.sub(initialCapturePoint, initialCMP);
         desiredCapturePointVelocityToPack.scale(omega0 * Math.exp(omega0 * time));
      }
      else
         desiredCapturePointVelocityToPack.setToZero();
   }

   /**
    * Compute the desired capture point velocity at a given time. ICPv_d = w * * e^{w*t} * ICP0 - p0
    * * w * e^{w*t}
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointVelocityToPack
    */
   public static void computeDesiredCapturePointVelocity(double omega0, double time, FramePoint3D initialCapturePoint, FramePoint3D initialCMP,
                                                         YoFrameVector desiredCapturePointVelocityToPack)
   {
      if (initialCapturePoint.distance(initialCMP) > EPSILON)
      {
         desiredCapturePointVelocityToPack.sub(initialCapturePoint, initialCMP);
         desiredCapturePointVelocityToPack.scale(omega0 * Math.exp(omega0 * time));
      }
      else
         desiredCapturePointVelocityToPack.setToZero();
   }

   /**
    * Compute the desired capture point acceleration given the desired capture point velocity
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param desiredCapturePointVelocity
    * @param desiredCapturePointAccelerationToPack
    */
   public static void computeDesiredCapturePointAcceleration(double omega0, YoFrameVector desiredCapturePointVelocity,
                                                             YoFrameVector desiredCapturePointAccelerationToPack)
   {
      desiredCapturePointAccelerationToPack.setAndMatchFrame(desiredCapturePointVelocity.getFrameTuple());
      desiredCapturePointAccelerationToPack.scale(omega0);
   }

   /**
    * Compute the desired capture point velocity at a given time. ICPv_d = w^2 * e^{w*t} * ICP0 - p0
    * * w^2 * e^{w*t}
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointAccelerationToPack
    */
   public static void computeDesiredCapturePointAcceleration(double omega0, double time, YoFramePoint initialCapturePoint, YoFramePoint initialCMP,
                                                             YoFrameVector desiredCapturePointAccelerationToPack)
   {
      if (initialCapturePoint.distance(initialCMP) > EPSILON)
      {
         desiredCapturePointAccelerationToPack.sub(initialCapturePoint, initialCMP);
         desiredCapturePointAccelerationToPack.scale(omega0 * omega0 * Math.exp(omega0 * time));
      }
      else
         desiredCapturePointAccelerationToPack.setToZero();
   }

   /**
    * Compute the desired capture point velocity at a given time. ICPv_d = w^2 * e^{w*t} * ICP0 - p0
    * * w^2 * e^{w*t}
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointAccelerationToPack
    */
   public static void computeDesiredCapturePointAcceleration(double omega0, double time, FramePoint3D initialCapturePoint, FramePoint3D initialCMP,
                                                             FrameVector3D desiredCapturePointAccelerationToPack)
   {
      if (initialCapturePoint.distance(initialCMP) > EPSILON)
      {
         desiredCapturePointAccelerationToPack.sub(initialCapturePoint, initialCMP);
         desiredCapturePointAccelerationToPack.scale(omega0 * omega0 * Math.exp(omega0 * time));
      }
      else
         desiredCapturePointAccelerationToPack.setToZero();
   }

   /**
    * Compute the desired capture point velocity at a given time. ICPv_d = w^2 * e^{w*t} * ICP0 - p0
    * * w^2 * e^{w*t}
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointAccelerationToPack
    */
   public static void computeDesiredCapturePointAcceleration(double omega0, double time, FramePoint3D initialCapturePoint, FramePoint3D initialCMP,
                                                             YoFrameVector desiredCapturePointAccelerationToPack)
   {
      if (initialCapturePoint.distance(initialCMP) > EPSILON)
      {
         desiredCapturePointAccelerationToPack.sub(initialCapturePoint, initialCMP);
         desiredCapturePointAccelerationToPack.scale(omega0 * omega0 * Math.exp(omega0 * time));
      }
      else
         desiredCapturePointAccelerationToPack.setToZero();
   }

   /**
    * Computes the desired centroidal momentum pivot by, CMP_{d} = ICP_{d} - \dot{ICP}_{d}/omega0
    *
    * @param desiredCapturePointPosition
    * @param desiredCapturePointVelocity
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param desiredCMPToPack
    */
   public static void computeDesiredCentroidalMomentumPivot(YoFramePoint desiredCapturePointPosition, YoFrameVector desiredCapturePointVelocity, double omega0,
                                                            YoFramePoint desiredCMPToPack)
   {
      desiredCMPToPack.scaleAdd(-1.0 / omega0, desiredCapturePointVelocity, desiredCapturePointPosition);
   }

   /**
    * Computes the desired centroidal momentum pivot velocity by, \dot{CMP}_{d} = \dot{ICP}_{d} -
    * \ddot{ICP}_{d}/omega0
    *
    * @param desiredCapturePointVelocity
    * @param desiredCapturePointAcceleration
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param desiredCMPVelocityToPack
    */
   public static void computeDesiredCentroidalMomentumPivotVelocity(YoFrameVector desiredCapturePointVelocity, YoFrameVector desiredCapturePointAcceleration,
                                                                    double omega0, YoFrameVector desiredCMPVelocityToPack)
   {
      desiredCMPVelocityToPack.scaleAdd(-1.0 / omega0, desiredCapturePointAcceleration, desiredCapturePointVelocity);
   }

   /**
    * Computes the desired centroidal momentum pivot by, CMP_{d} = ICP_{d} - \dot{ICP}_{d}/omega0
    *
    * @param desiredCapturePointPosition
    * @param desiredCapturePointVelocity
    * @param omega0
    * @param desiredCMPToPack
    */
   public static void computeDesiredCentroidalMomentumPivot(YoFramePoint2d desiredCapturePointPosition, YoFrameVector2d desiredCapturePointVelocity,
                                                            double omega0, YoFramePoint2d desiredCMPToPack)
   {
      desiredCMPToPack.scaleAdd(-1.0 / omega0, desiredCapturePointVelocity, desiredCapturePointPosition);
   }

   /**
    * Computes the desired centroidal momentum pivot by, CMP_{d} = ICP_{d} - \dot{ICP}_{d}/omega0
    *
    * @param desiredCapturePointPosition
    * @param desiredCapturePointVelocity
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param desiredCMPToPack
    */
   public static void computeDesiredCentroidalMomentumPivot(FramePoint3D desiredCapturePointPosition, FrameVector3D desiredCapturePointVelocity, double omega0,
                                                            YoFramePoint desiredCMPToPack)
   {
      desiredCMPToPack.scaleAdd(-1.0 / omega0, desiredCapturePointVelocity, desiredCapturePointPosition);
   }

   /**
    * Computes the desired centroidal momentum pivot by, CMP_{d} = ICP_{d} - \dot{ICP}_{d}/omega0
    *
    * @param desiredCapturePointPosition
    * @param desiredCapturePointVelocity
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param desiredCMPToPack
    */
   public static void computeDesiredCentroidalMomentumPivot(FramePoint2D desiredCapturePointPosition, FrameVector2D desiredCapturePointVelocity, double omega0,
                                                            FramePoint2D desiredCMPToPack)
   {
      desiredCMPToPack.scaleAdd(-1.0 / omega0, desiredCapturePointVelocity, desiredCapturePointPosition);
   }

   /**
    * Compute the distance along the capture point guide line from the current capture point
    * position to the desired capture point position.
    *
    * @param currentCapturePointPosition
    * @param desiredCapturePointPosition
    * @param desiredCapturePointVelocity
    * @return
    */
   public static double computeDistanceToCapturePointFreezeLineIn2d(FramePoint3D currentCapturePointPosition, FramePoint3D desiredCapturePointPosition,
                                                                    FrameVector3D desiredCapturePointVelocity)
   {
      currentCapturePointPosition.checkReferenceFrameMatch(desiredCapturePointPosition);
      desiredCapturePointVelocity.checkReferenceFrameMatch(desiredCapturePointPosition);

      double desiredCapturePointVelocityMagnitude = Math.sqrt(MathTools.square(desiredCapturePointVelocity.getX())
            + MathTools.square(desiredCapturePointVelocity.getY()));

      if (desiredCapturePointVelocityMagnitude == 0.0)
      {
         return Double.NaN;
      }
      else
      {
         double normalizedCapturePointVelocityX = desiredCapturePointVelocity.getX() / desiredCapturePointVelocityMagnitude;
         double normalizedCapturePointVelocityY = desiredCapturePointVelocity.getY() / desiredCapturePointVelocityMagnitude;

         double capturePointErrorX = currentCapturePointPosition.getX() - desiredCapturePointPosition.getX();
         double capturePointErrorY = currentCapturePointPosition.getY() - desiredCapturePointPosition.getY();

         return -(normalizedCapturePointVelocityX * capturePointErrorX + normalizedCapturePointVelocityY * capturePointErrorY);
      }
   }

   /**
    * Compute the distance along the capture point guide line from the current capture point
    * position to the desired capture point position.
    *
    * @param currentCapturePointPosition
    * @param desiredCapturePointPosition
    * @param desiredCapturePointVelocity
    * @return
    */
   public static double computeDistanceToCapturePointFreezeLineIn2d(FramePoint2D currentCapturePointPosition, FramePoint2D desiredCapturePointPosition,
                                                                    FrameVector2D desiredCapturePointVelocity)
   {
      currentCapturePointPosition.checkReferenceFrameMatch(desiredCapturePointPosition);
      desiredCapturePointVelocity.checkReferenceFrameMatch(desiredCapturePointPosition);

      double desiredCapturePointVelocityMagnitude = Math.sqrt(MathTools.square(desiredCapturePointVelocity.getX())
            + MathTools.square(desiredCapturePointVelocity.getY()));

      if (desiredCapturePointVelocityMagnitude == 0.0)
      {
         return Double.NaN;
      }
      else
      {
         double normalizedCapturePointVelocityX = desiredCapturePointVelocity.getX() / desiredCapturePointVelocityMagnitude;
         double normalizedCapturePointVelocityY = desiredCapturePointVelocity.getY() / desiredCapturePointVelocityMagnitude;

         double capturePointErrorX = currentCapturePointPosition.getX() - desiredCapturePointPosition.getX();
         double capturePointErrorY = currentCapturePointPosition.getY() - desiredCapturePointPosition.getY();

         return -(normalizedCapturePointVelocityX * capturePointErrorX + normalizedCapturePointVelocityY * capturePointErrorY);
      }
   }

   /**
    * Given a capture point trajectory line and the actual capture point position, project the
    * current capture point onto the capture point trajectory line.
    *
    * @param actualICP
    * @param capturePointTrajectoryLine
    * @param projectedCapturePoint
    */
   public static void computeCapturePointOnTrajectoryAndClosestToActualCapturePoint(FramePoint3D actualICP, FrameLine2d capturePointTrajectoryLine,
                                                                                    FramePoint2D projectedCapturePoint)
   {
      projectedCapturePoint.set(actualICP.getX(), actualICP.getY());
      capturePointTrajectoryLine.orthogonalProjection(projectedCapturePoint);
   }

   /**
    * Compute the capture point position at a given time. x<sup>ICP<sub>des</sub></sup> =
    * (e<sup>&omega;0 t</sup>) x<sup>ICP<sub>0</sub></sup> + (1-e<sup>&omega;0
    * t</sup>)x<sup>CMP<sub>0</sub></sup>
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointToPack
    */
   public static void computeCapturePointPosition(double omega0, double time, FramePoint2D initialCapturePoint, FramePoint2D initialCMP,
                                                  FramePoint2D desiredCapturePointToPack)
   {
      desiredCapturePointToPack.setToZero(initialCapturePoint.getReferenceFrame());

      if (initialCapturePoint.distance(initialCMP) > EPSILON)
         desiredCapturePointToPack.interpolate(initialCMP, initialCapturePoint, Math.exp(omega0 * time));
      else
         desiredCapturePointToPack.set(initialCapturePoint);
   }

   /**
    * Compute the angular momentum about the center of mass in the world frame.
    * x<sup>eCMP</sup> = x<sup>ZMP</sup> + 1 / (m g ) * [&tau;<sup>y</sup>,
    * -&tau;<sup>x</sup>]. Assumes no acceleration in the vertical direction.
    *
    * @param mass total mass of the robot
    * @param gravity gravityZ
    * @param eCMP location of the eCMP
    * @param zmp location of the ZMP
    * @param angularMomentumToPack angular momentum about the center of mass. Modified.
    */
   public static void computeAngularMomentum(double mass, double gravity, FramePoint2D eCMP, FramePoint2D zmp, FramePoint2D angularMomentumToPack)
   {
      computeAngularMomentum(mass, gravity, 0.0, eCMP, zmp, angularMomentumToPack);
   }

   /**
    * Compute the angular momentum about the center of mass in the world frame.
    * x<sup>eCMP</sup> = x<sup>ZMP</sup> + 1 / (m (g + \ddot{z}<sup>CoM</sup>)) * [&tau;<sup>y</sup>,
    * -&tau;<sup>x</sup>].
    *
    * @param mass total mass of the robot
    * @param gravity gravityZ
    * @param comAcceleration acceleration of the center of mass
    * @param eCMP location of the eCMP
    * @param zmp location of the ZMP
    * @param angularMomentumToPack angular momentum about the center of mass. Modified.
    */
   public static void computeAngularMomentum(double mass, double gravity, FrameVector3D comAcceleration, FramePoint2D eCMP, FramePoint2D zmp, FramePoint2D angularMomentumToPack)
   {
      comAcceleration.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      computeAngularMomentum(mass, gravity, comAcceleration.getZ(), eCMP, zmp, angularMomentumToPack);
   }

   /**
    * Compute the angular momentum about the center of mass in the world frame.
    * x<sup>eCMP</sup> = x<sup>ZMP</sup> + 1 / (m (g + \ddot{z}<sup>CoM</sup>)) * [&tau;<sup>y</sup>,
    * -&tau;<sup>x</sup>].
    *
    * @param mass total mass of the robot
    * @param gravity gravityZ
    * @param comVerticalAcceleration acceleration of the center of mass in the vertical direction
    * @param eCMP location of the eCMP
    * @param zmp location of the ZMP
    * @param angularMomentumToPack angular momentum about the center of mass. Modified.
    */
   public static void computeAngularMomentum(double mass, double gravity, double comVerticalAcceleration, FramePoint2D eCMP, FramePoint2D zmp, FramePoint2D angularMomentumToPack)
   {
      angularMomentumToPack.setToZero(ReferenceFrame.getWorldFrame());
      angularMomentumToPack.set(eCMP);
      angularMomentumToPack.sub(zmp);

      double xValue = angularMomentumToPack.getX();
      double yValue = angularMomentumToPack.getY();

      angularMomentumToPack.setX(yValue);
      angularMomentumToPack.setY(-xValue);
      angularMomentumToPack.scale(1.0 / (mass * (gravity + comVerticalAcceleration)));
   }

   /**
    * Compute the angular momentum about the center of mass in the world frame.
    * x<sup>eCMP</sup> = x<sup>ZMP</sup> + 1 / (m g ) * [&tau;<sup>y</sup>,
    * -&tau;<sup>x</sup>]. Assumes no acceleration in the vertical direction.
    *
    * @param mass total mass of the robot
    * @param gravity gravityZ
    * @param delta distance between the eCMP and the ZMP.
    * @param angularMomentumToPack angular momentum about the center of mass. Modified.
    */
   public static void computeAngularMomentum(double mass, double gravity, FramePoint2D delta, FramePoint2D angularMomentumToPack)
   {
      computeAngularMomentum(mass, gravity, 0.0, delta, angularMomentumToPack);
   }

   /**
    * Compute the angular momentum about the center of mass in the world frame.
    * x<sup>eCMP</sup> = x<sup>ZMP</sup> + 1 / (m (g + \ddot{z}<sup>CoM</sup>)) * [&tau;<sup>y</sup>,
    * -&tau;<sup>x</sup>].
    *
    * @param mass total mass of the robot
    * @param gravity gravityZ
    * @param comVerticalAcceleration acceleration of the center of mass in the vertical direction
    * @param delta distance between the eCMP and the ZMP.
    * @param angularMomentumToPack angular momentum about the center of mass. Modified.
    */
   public static void computeAngularMomentum(double mass, double gravity, double comVerticalAcceleration, FramePoint2D delta, FramePoint2D angularMomentumToPack)
   {
      delta.changeFrame(ReferenceFrame.getWorldFrame());

      double xValue = delta.getX();
      double yValue = delta.getY();

      angularMomentumToPack.setX(yValue);
      angularMomentumToPack.setY(-xValue);
      angularMomentumToPack.scale(1.0 / (mass * (gravity + comVerticalAcceleration)));
   }

}
