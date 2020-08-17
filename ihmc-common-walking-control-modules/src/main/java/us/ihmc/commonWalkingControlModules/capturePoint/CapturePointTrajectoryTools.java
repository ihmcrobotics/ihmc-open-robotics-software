package us.ihmc.commonWalkingControlModules.capturePoint;

import java.util.List;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoDouble;

public class CapturePointTrajectoryTools
{
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
   public static void computeConstantCMPs(List<YoFramePoint3D> constantCMPsToPack, List<? extends FramePoint3DReadOnly> footstepList, int firstFootstepIndex,
                                          int lastFootstepIndex, boolean startStanding, boolean endStanding)
   {
      if (startStanding)
      {
         // Start with the first constant CMP located between the feet.
         YoFramePoint3D firstConstantCMPPlanned = constantCMPsToPack.get(firstFootstepIndex);
         FramePoint3DReadOnly firstFootstepToConsider = footstepList.get(firstFootstepIndex);
         FramePoint3DReadOnly secondFootstepToConsider = footstepList.get(firstFootstepIndex + 1);
         putConstantCMPBetweenFeet(firstConstantCMPPlanned, firstFootstepToConsider, secondFootstepToConsider);
         firstFootstepIndex++;
      }

      if (endStanding)
      {
         // End with the last constant CMP located between the feet.
         YoFramePoint3D lastConstantCMPPlanned = constantCMPsToPack.get(lastFootstepIndex);
         FramePoint3DReadOnly lastFootstepToConsider = footstepList.get(lastFootstepIndex);
         FramePoint3DReadOnly beforeLastFootstepToConsider = footstepList.get(lastFootstepIndex - 1);
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
   public static void computeConstantCMPsOnFeet(List<YoFramePoint3D> constantCMPsToPack, List<? extends FramePoint3DReadOnly> footstepList,
                                                int firstFootstepIndex, int lastFootstepIndex)
   {
      for (int i = firstFootstepIndex; i <= lastFootstepIndex; i++)
      {
         YoFramePoint3D constantCMP = constantCMPsToPack.get(i);
         // Put the constant CMP at the footstep location
         constantCMP.setMatchingFrame(footstepList.get(i));
      }
   }

   /**
    * Put the constant CMP in the middle of the two given footsteps.
    *
    * @param constantCMPToPack YoFramePoint that will be packed with the constant CMP location
    * @param firstFootstep FramePoint holding the position of the first footstep
    * @param secondFootstep FramePoint holding the position of the second footstep
    */
   public static void putConstantCMPBetweenFeet(YoFramePoint3D constantCMPToPack, FramePoint3DReadOnly firstFootstep, FramePoint3DReadOnly secondFootstep)
   {
      constantCMPToPack.setMatchingFrame(firstFootstep);
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
   public static void computeDesiredCornerPoints(List<? extends FixedFramePoint3DBasics> cornerPointsToPack, List<? extends FramePoint3DReadOnly> constantCMPs,
                                                 boolean skipFirstCornerPoint, double stepTime, double omega0)
   {
      double exponentialTerm = Math.exp(-omega0 * stepTime);
      FramePoint3DReadOnly nextCornerPoint = constantCMPs.get(cornerPointsToPack.size());

      int firstCornerPointIndex = skipFirstCornerPoint ? 1 : 0;
      for (int i = cornerPointsToPack.size() - 1; i >= firstCornerPointIndex; i--)
      {
         FixedFramePoint3DBasics cornerPoint = cornerPointsToPack.get(i);
         FramePoint3DReadOnly initialCMP = constantCMPs.get(i);

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
   public static void computeDesiredCornerPointsDoubleSupport(List<? extends FixedFramePoint3DBasics> cornerPointsToPack,
                                                              List<? extends FramePoint3DReadOnly> constantCMPs, List<YoDouble> swingTimes,
                                                              List<YoDouble> transferTimes, double doubleSupportSplitFraction, double omega0)
   {
      FramePoint3DReadOnly nextCornerPoint = constantCMPs.get(cornerPointsToPack.size());

      for (int i = cornerPointsToPack.size() - 1; i >= 1; i--)
      {
         double stepTime = swingTimes.get(i - 1).getDoubleValue();
         stepTime += (1.0 - doubleSupportSplitFraction) * transferTimes.get(i - 1).getDoubleValue();
         stepTime += doubleSupportSplitFraction * transferTimes.get(i).getDoubleValue();

         double exponentialTerm = Math.exp(-omega0 * stepTime);

         FixedFramePoint3DBasics cornerPoint = cornerPointsToPack.get(i);
         FramePoint3DReadOnly initialCMP = constantCMPs.get(i);

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
   public static void computeDesiredCornerPointsDoubleSupport(List<? extends FixedFramePoint3DBasics> cornerPointsToPack,
                                                              List<? extends FramePoint3DReadOnly> constantCMPs, List<YoDouble> swingTimes,
                                                              List<YoDouble> transferTimes, List<YoDouble> doubleSupportSplitFractions, double omega0)
   {
      FramePoint3DReadOnly nextCornerPoint = constantCMPs.get(cornerPointsToPack.size());

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

         FixedFramePoint3DBasics cornerPoint = cornerPointsToPack.get(i);
         FramePoint3DReadOnly initialCMP = constantCMPs.get(i);

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
   public static void computeDesiredCornerPointsSingleSupport(List<? extends FixedFramePoint3DBasics> cornerPointsToPack,
                                                              List<? extends FramePoint3DReadOnly> constantCMPs, List<YoDouble> swingTimes,
                                                              List<YoDouble> transferTimes, double doubleSupportSplitFraction, double omega0)
   {
      FramePoint3DReadOnly nextCornerPoint = constantCMPs.get(cornerPointsToPack.size());

      for (int i = cornerPointsToPack.size() - 1; i >= 0; i--)
      {
         double stepTime = swingTimes.get(i).getDoubleValue();
         stepTime += (1.0 - doubleSupportSplitFraction) * transferTimes.get(i).getDoubleValue();
         stepTime += doubleSupportSplitFraction * transferTimes.get(i + 1).getDoubleValue();

         FixedFramePoint3DBasics cornerPoint = cornerPointsToPack.get(i);
         FramePoint3DReadOnly initialCMP = constantCMPs.get(i);

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
   public static void computeDesiredCornerPointsSingleSupport(List<? extends FixedFramePoint3DBasics> cornerPointsToPack,
                                                              List<? extends FramePoint3DReadOnly> constantCMPs, List<YoDouble> swingTimes,
                                                              List<YoDouble> transferTimes, List<YoDouble> doubleSupportSplitFractions, double omega0)
   {
      FramePoint3DReadOnly nextCornerPoint = constantCMPs.get(cornerPointsToPack.size());

      for (int i = cornerPointsToPack.size() - 1; i >= 0; i--)
      {
         double doubleSupportSplitFraction = doubleSupportSplitFractions.get(i).getDoubleValue();
         double nextDoubleSupportSplitFraction = doubleSupportSplitFractions.get(i + 1).getDoubleValue();

         double transferTime = transferTimes.get(i).getDoubleValue();
         double nextTransferTime = transferTimes.get(i + 1).getDoubleValue();
         double stepTime = swingTimes.get(i).getDoubleValue();
         stepTime += (1.0 - doubleSupportSplitFraction) * transferTime;
         stepTime += nextDoubleSupportSplitFraction * nextTransferTime;

         FixedFramePoint3DBasics cornerPoint = cornerPointsToPack.get(i);
         FramePoint3DReadOnly initialCMP = constantCMPs.get(i);

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
   public static void computeDesiredCornerPoints(List<? extends FixedFramePoint3DBasics> entryCornerPointsToPack,
                                                 List<? extends FixedFramePoint3DBasics> exitCornerPointsToPack,
                                                 List<? extends FramePoint3DReadOnly> entryCMPs, List<? extends FramePoint3DReadOnly> exitCMPs, double stepTime,
                                                 double timeInPercentSpentOnExitCMPs, double omega0)
   {
      double timeSpentOnExitCMP = stepTime * timeInPercentSpentOnExitCMPs;
      double timeSpentOnEntryCMP = stepTime * (1.0 - timeInPercentSpentOnExitCMPs);
      double entryExponentialTerm = Math.exp(-omega0 * timeSpentOnEntryCMP);
      double exitExponentialTerm = Math.exp(-omega0 * timeSpentOnExitCMP);

      FramePoint3DReadOnly nextEntryCornerPoint = entryCMPs.get(entryCornerPointsToPack.size());

      for (int i = exitCornerPointsToPack.size() - 1; i >= 0; i--)
      {
         FixedFramePoint3DBasics exitCornerPoint = exitCornerPointsToPack.get(i);
         FixedFramePoint3DBasics entryCornerPoint = entryCornerPointsToPack.get(i);
         FramePoint3DReadOnly exitCMP = exitCMPs.get(i);
         FramePoint3DReadOnly entryCMP = entryCMPs.get(i);

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
   public static void computeDesiredCornerPointsDoubleSupport(List<? extends FixedFramePoint3DBasics> entryCornerPointsToPack,
                                                              List<? extends FixedFramePoint3DBasics> exitCornerPointsToPack,
                                                              List<? extends FramePoint3DReadOnly> entryCMPs, List<? extends FramePoint3DReadOnly> exitCMPs,
                                                              List<YoDouble> swingTimes, List<YoDouble> transferTimes, double swingSplitFraction,
                                                              double transferSplitFraction, double omega0)
   {
      FramePoint3DReadOnly nextEntryCornerPoint = entryCMPs.get(entryCornerPointsToPack.size());

      for (int i = exitCornerPointsToPack.size() - 1; i >= 1; i--)
      {
         double upcomingSwingTime = swingTimes.get(i - 1).getDoubleValue();
         double transferTime = transferTimes.get(i - 1).getDoubleValue();
         double nextTransferTime = transferTimes.get(i).getDoubleValue();

         double timeSpentOnEntryCMP = upcomingSwingTime * swingSplitFraction + transferTime * (1.0 - transferSplitFraction);
         double timeSpentOnExitCMP = upcomingSwingTime * (1.0 - swingSplitFraction) + nextTransferTime * transferSplitFraction;
         double entryExponentialTerm = Math.exp(-omega0 * timeSpentOnEntryCMP);
         double exitExponentialTerm = Math.exp(-omega0 * timeSpentOnExitCMP);

         FixedFramePoint3DBasics exitCornerPoint = exitCornerPointsToPack.get(i);
         FixedFramePoint3DBasics entryCornerPoint = entryCornerPointsToPack.get(i);
         FramePoint3DReadOnly exitCMP = exitCMPs.get(i);
         FramePoint3DReadOnly entryCMP = entryCMPs.get(i);

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
   public static void computeDesiredCornerPointsDoubleSupport(List<? extends FixedFramePoint3DBasics> entryCornerPointsToPack,
                                                              List<? extends FixedFramePoint3DBasics> exitCornerPointsToPack,
                                                              List<? extends FramePoint3DReadOnly> entryCMPs, List<? extends FramePoint3DReadOnly> exitCMPs,
                                                              List<YoDouble> swingTimes, List<YoDouble> transferTimes, List<YoDouble> swingSplitFractions,
                                                              List<YoDouble> transferSplitFractions, double omega0)
   {
      FramePoint3DReadOnly nextEntryCornerPoint = entryCMPs.get(entryCornerPointsToPack.size());

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

         FixedFramePoint3DBasics exitCornerPoint = exitCornerPointsToPack.get(i);
         FixedFramePoint3DBasics entryCornerPoint = entryCornerPointsToPack.get(i);
         FramePoint3DReadOnly exitCMP = exitCMPs.get(i);
         FramePoint3DReadOnly entryCMP = entryCMPs.get(i);

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
   public static void computeDesiredCornerPointsSingleSupport(List<? extends FixedFramePoint3DBasics> entryCornerPointsToPack,
                                                              List<? extends FixedFramePoint3DBasics> exitCornerPointsToPack,
                                                              List<? extends FramePoint3DReadOnly> entryCMPs, List<? extends FramePoint3DReadOnly> exitCMPs,
                                                              List<YoDouble> swingTimes, List<YoDouble> transferTimes, double swingSplitFraction,
                                                              double transferSplitFraction, double omega0)
   {
      FramePoint3DReadOnly nextEntryCornerPoint = entryCMPs.get(entryCornerPointsToPack.size());

      for (int i = exitCornerPointsToPack.size() - 1; i >= 0; i--)
      {
         double swingTime = swingTimes.get(i).getDoubleValue();
         double previousTransferTime = transferTimes.get(i).getDoubleValue();
         double nextTransferTime = transferTimes.get(i + 1).getDoubleValue();

         FixedFramePoint3DBasics exitCornerPoint = exitCornerPointsToPack.get(i);
         FixedFramePoint3DBasics entryCornerPoint = entryCornerPointsToPack.get(i);
         FramePoint3DReadOnly exitCMP = exitCMPs.get(i);
         FramePoint3DReadOnly entryCMP = entryCMPs.get(i);

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
   public static void computeDesiredCornerPointsSingleSupport(List<? extends FixedFramePoint3DBasics> entryCornerPointsToPack,
                                                              List<? extends FixedFramePoint3DBasics> exitCornerPointsToPack,
                                                              List<? extends FramePoint3DReadOnly> entryCMPs, List<? extends FramePoint3DReadOnly> exitCMPs,
                                                              List<YoDouble> swingTimes, List<YoDouble> transferTimes, List<YoDouble> swingSplitFractions,
                                                              List<YoDouble> transferSplitFractions, double omega0)
   {
      FramePoint3DReadOnly nextEntryCornerPoint = entryCMPs.get(entryCornerPointsToPack.size());

      for (int i = exitCornerPointsToPack.size() - 1; i >= 0; i--)
      {
         double swingSplitFraction = swingSplitFractions.get(i).getDoubleValue();
         double previousTransferSplitFraction = transferSplitFractions.get(i).getDoubleValue();
         double nextTransferSplitFraction = transferSplitFractions.get(i + 1).getDoubleValue();

         double swingTime = swingTimes.get(i).getDoubleValue();
         double previousTransferTime = transferTimes.get(i).getDoubleValue();
         double nextTransferTime = transferTimes.get(i + 1).getDoubleValue();

         FixedFramePoint3DBasics exitCornerPoint = exitCornerPointsToPack.get(i);
         FixedFramePoint3DBasics entryCornerPoint = entryCornerPointsToPack.get(i);
         FramePoint3DReadOnly exitCMP = exitCMPs.get(i);
         FramePoint3DReadOnly entryCMP = entryCMPs.get(i);

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
   public static void computeConstantCMPFromInitialAndFinalCapturePointLocations(FixedFramePoint3DBasics cmpToPack, FramePoint3DReadOnly finalDesiredCapturePoint,
                                                                                 FramePoint3DReadOnly initialCapturePoint, double omega0, double stepTime)
   {
      double exponentialTerm = Math.exp(-omega0 * stepTime);
      cmpToPack.scaleSub(exponentialTerm, finalDesiredCapturePoint, initialCapturePoint);
      cmpToPack.scale(1.0 / (exponentialTerm - 1.0));
   }
}
