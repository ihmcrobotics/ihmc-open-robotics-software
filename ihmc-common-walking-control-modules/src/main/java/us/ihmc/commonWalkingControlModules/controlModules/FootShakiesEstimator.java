package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.filters.AlphaFilterTools;
import us.ihmc.yoVariables.filters.FilteredFiniteDifferenceYoVariable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class FootShakiesEstimator
{
   private static final double COP_WINDOW_DURATION = 0.5;
   private static final double COP_SEGMENT_GLITCH_DURATION = 0.025;
   private static final double COP_MIDDLE_SEGMENT_FRACTION = 0.6;
   private static final double MIN_WEIGHT_FRACTION_FOR_CONTROL_SCALING = 0.32;
   private static final int COP_FORWARD_SWITCHES_FOR_SHAKIES = 4;
   private static final int COP_SIDEWAYS_SWITCHES_FOR_SHAKIES = 3;


   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FramePoint2D copDesired = new FramePoint2D();
   private final FramePoint2D copActual = new FramePoint2D();
   private final Wrench footWrench = new Wrench();
   private final FrameVector3D footForceVector = new FrameVector3D();

   // State containers used by this module
   private final SideDependentList<ContactableFoot> feet;
   private final SideDependentList<? extends FootSwitchInterface> footSwitches;
   private final DoubleProvider time;

   // Variables to control Foot shakies
   private final YoBoolean enableHighCoPDampingForShakies = new YoBoolean("enableHighCoPDampingForShakies", registry);
   private final YoBoolean isCoPTrackingBad = new YoBoolean("isCoPTrackingBad", registry);
   private final YoBoolean isCoPDamped = new YoBoolean("isCoPDamped", registry);
   private final YoDouble highCoPDampingErrorTrigger = new YoDouble("highCoPDampingErrorTrigger", registry);
   private final YoDouble highCoPDampingStartTime = new YoDouble("highCoPDampingStartTime", registry);
   private final YoDouble highCoPDampingDuration = new YoDouble("highCoPDampingDuration", registry);

   private final YoInteger copForwardSwitchesForShakies = new YoInteger("copForwardSwitchesForShakies", registry);
   private final YoInteger copSidewaysSwitchesForShakies = new YoInteger("copSidewaysSwitchesForShakies", registry);

   private final DoubleProvider totalMass;
   private final double gravityZ;

   // variables for computing how many times it switches sides
   private final YoDouble copWindowDuration = new YoDouble("copWindowLength", registry);
   private final YoDouble copSegmentGlitchDuration = new YoDouble("copSegmentGlitchDuration", registry);
   private final YoDouble copMiddleSegmentFraction = new YoDouble("copMiddleSegmentFraction", registry);
   private final YoDouble minWeightFractionForControlScaling = new YoDouble("minWeightFractionForControlScaling", registry);

   private final SideDependentList<YoInteger> copForwardSegments = new SideDependentList<>();
   private final SideDependentList<YoInteger> copSidewaysSegments = new SideDependentList<>();
   private final SideDependentList<YoInteger> copForwardGlitchCounters = new SideDependentList<>();
   private final SideDependentList<YoInteger> copSidewaysGlitchCounters = new SideDependentList<>();
   private final SideDependentList<YoInteger> copForwardSwitches = new SideDependentList<>();
   private final SideDependentList<YoInteger> copSidewaysSwitches = new SideDependentList<>();

   private final SideDependentList<int[]> copInForwardWindows = new SideDependentList<>();
   private final SideDependentList<int[]> copInSidewaysWindows = new SideDependentList<>();
   private final SideDependentList<int[]> filteredCopInForwardWindows = new SideDependentList<>();
   private final SideDependentList<int[]> filteredCopInSidewaysWindows = new SideDependentList<>();

   // Variables computed by this estimator
   private final SideDependentList<YoFrameVector2D> yoCoPError = new SideDependentList<>();
   private final SideDependentList<YoDouble> yoCoPErrorMagnitude = new SideDependentList<YoDouble>(new YoDouble("leftFootCoPErrorMagnitude", registry),
                                                                                                   new YoDouble("rightFootCoPErrorMagnitude", registry));
   private final YoDouble copRateBreakFrequency = new YoDouble("copRateBreakFrequency", registry);
   private final SideDependentList<FilteredFiniteDifferenceYoVariable> yoCoPErrorRate = new SideDependentList<>();

   private final double footLength;
   private final double footWidth;

   private final double controlDT;
   private int writeIndex = -1;
   private int windowCurrentSize = 0;
   private final int windowMaxSize;

   public FootShakiesEstimator(SideDependentList<ContactableFoot> feet,
                               SideDependentList<? extends FootSwitchInterface> footSwitches,
                               DoubleProvider time,
                               DoubleProvider totalMass,
                               double controlDT,
                               double gravityZ,
                               YoRegistry parentRegistry)
   {
      this.feet = feet;
      this.footSwitches = footSwitches;
      this.time = time;
      this.controlDT = controlDT;
      this.totalMass = totalMass;
      this.gravityZ = gravityZ;

      copWindowDuration.set(COP_WINDOW_DURATION);
      copSegmentGlitchDuration.set(COP_SEGMENT_GLITCH_DURATION);
      copMiddleSegmentFraction.set(COP_MIDDLE_SEGMENT_FRACTION);
      windowMaxSize = (int) Math.ceil(copWindowDuration.getDoubleValue() / controlDT);
      minWeightFractionForControlScaling.set(MIN_WEIGHT_FRACTION_FOR_CONTROL_SCALING);
      copForwardSwitchesForShakies.set(COP_FORWARD_SWITCHES_FOR_SHAKIES);
      copSidewaysSwitchesForShakies.set(COP_SIDEWAYS_SWITCHES_FOR_SHAKIES);

      for (RobotSide robotSide : RobotSide.values)
      {
         copForwardSegments.put(robotSide, new YoInteger(robotSide.getCamelCaseNameForStartOfExpression() + "CopForwardSegment", registry));
         copSidewaysSegments.put(robotSide, new YoInteger(robotSide.getCamelCaseNameForStartOfExpression() + "CopSidewaysSegment", registry));
         copForwardGlitchCounters.put(robotSide, new YoInteger(robotSide.getCamelCaseNameForStartOfExpression() + "CopForwardGlitchCounter", registry));
         copSidewaysGlitchCounters.put(robotSide, new YoInteger(robotSide.getCamelCaseNameForStartOfExpression() + "CopSidewaysGlitchCounter", registry));
         copForwardSwitches.put(robotSide, new YoInteger(robotSide.getCamelCaseNameForStartOfExpression() + "CopForwardSwitches", registry));
         copSidewaysSwitches.put(robotSide, new YoInteger(robotSide.getCamelCaseNameForStartOfExpression() + "CopSidewaysSwitches", registry));

         copInForwardWindows.put(robotSide, new int[windowMaxSize]);
         copInSidewaysWindows.put(robotSide, new int[windowMaxSize]);
         filteredCopInForwardWindows.put(robotSide, new int[windowMaxSize]);
         filteredCopInSidewaysWindows.put(robotSide, new int[windowMaxSize]);
      }


      double maxX = Double.NEGATIVE_INFINITY;
      double minX = Double.POSITIVE_INFINITY;
      double maxY = Double.NEGATIVE_INFINITY;
      double minY = Double.POSITIVE_INFINITY;
      for (FramePoint2DReadOnly contactPoint : feet.get(RobotSide.LEFT).getContactPoints2D())
      {
         maxX = Math.max(maxX, contactPoint.getX());
         minX = Math.min(minX, contactPoint.getX());
         maxY = Math.max(maxY, contactPoint.getY());
         minY = Math.min(minY, contactPoint.getY());
      }
      footLength = maxX - minX;
      footWidth = maxY - minY;

      copRateBreakFrequency.set(30.0);
      DoubleProvider copRateAlpha = () -> AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(copRateBreakFrequency.getDoubleValue(), controlDT);

      for (RobotSide robotSide : RobotSide.values)
      {
         yoCoPError.put(robotSide,
                        new YoFrameVector2D(robotSide.getCamelCaseNameForStartOfExpression() + "FootCoPError", feet.get(robotSide).getContactFrame(), registry));
         yoCoPErrorRate.put(robotSide,
                            new FilteredFiniteDifferenceYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "FootCoPErrorRate", "", copRateAlpha, controlDT, registry));
      }

      parentRegistry.addChild(registry);
   }

   public void setHighCoPDampingParameters(boolean enable, double duration, double copErrorThreshold)
   {
      enableHighCoPDampingForShakies.set(enable);
      highCoPDampingDuration.set(duration);
      highCoPDampingErrorTrigger.set(copErrorThreshold);
   }

   public boolean isCoPDamped()
   {
      return isCoPDamped.getBooleanValue();
   }

   public boolean estimateIfHighCoPDampingNeeded(SideDependentList<? extends FramePoint2DReadOnly> desiredCoPs)
   {
      if (!enableHighCoPDampingForShakies.getBooleanValue())
      {
         isCoPDamped.set(false);
         return false;
      }

      boolean atLeastOneFootWithBadCoPControl = false;

      double minZForceForCoPControlScaling = minWeightFractionForControlScaling.getDoubleValue() * totalMass.getValue() * gravityZ;

      for (RobotSide robotSide : RobotSide.values)
      {
         copDesired.setIncludingFrame(desiredCoPs.get(robotSide));
         FootSwitchInterface footSwitch = footSwitches.get(robotSide);
         footSwitch.getCenterOfPressure(copActual);

         YoFrameVector2D copError = yoCoPError.get(robotSide);
         YoDouble copErrorMagnitude = yoCoPErrorMagnitude.get(robotSide);
         FilteredFiniteDifferenceYoVariable copErrorRate = yoCoPErrorRate.get(robotSide);

         if (Double.isNaN(copErrorRate.getDoubleValue()))
            copErrorRate.reset();

         if (copDesired.containsNaN() || copActual.containsNaN())
         {
            copError.setToZero();
            copErrorMagnitude.set(0.0);
            copErrorRate.set(0.0);
         }
         else
         {
            copError.sub(copDesired, copActual);
            copErrorMagnitude.set(copError.norm());
            copErrorRate.update(copErrorMagnitude.getDoubleValue());
         }

         footSwitch.getMeasuredWrench(footWrench);
         footForceVector.setIncludingFrame(footWrench.getLinearPart());
         footForceVector.changeFrame(ReferenceFrame.getWorldFrame());

         if (footForceVector.getZ() > minZForceForCoPControlScaling && copErrorMagnitude.getDoubleValue() > highCoPDampingErrorTrigger.getDoubleValue())
         {
            atLeastOneFootWithBadCoPControl = true;
         }
      }

      // Update the number of times the CoP has jumped back and forth
      updateCoPSwitchingCounters(desiredCoPs);

      for (RobotSide robotSide : RobotSide.values)
      {
         if (copForwardSwitches.get(robotSide).getValue() >= copForwardSwitchesForShakies.getValue())
            atLeastOneFootWithBadCoPControl = true;
         if (copSidewaysSwitches.get(robotSide).getValue() >= copSidewaysSwitchesForShakies.getValue())
            atLeastOneFootWithBadCoPControl = true;
      }

      isCoPTrackingBad.set(atLeastOneFootWithBadCoPControl);

      boolean isCoPDampened = time.getValue() - highCoPDampingStartTime.getDoubleValue() <= highCoPDampingDuration.getDoubleValue();

      if (atLeastOneFootWithBadCoPControl)
      {
         // Restart the clock because one still has bad control
         highCoPDampingStartTime.set(time.getValue());
         isCoPDampened = true;
      }

      this.isCoPDamped.set(isCoPDampened);
      return isCoPDampened;
   }

   private void updateCoPSwitchingCounters(SideDependentList<? extends FramePoint2DReadOnly> desiredCoPs)
   {
      int glitchFilterSize = (int) Math.ceil(copSegmentGlitchDuration.getDoubleValue() / controlDT);

      double minZForceForCoPControlScaling = minWeightFractionForControlScaling.getDoubleValue() * totalMass.getValue() * gravityZ;

      // Get the next index
      int oldIndex = writeIndex;
      writeIndex = nextWindowIndex(writeIndex);
      // Record whether the cop is in different segments of the foot.

      double xThreshold = footLength * copMiddleSegmentFraction.getValue() / 2.0;
      double yThreshold = footWidth * copMiddleSegmentFraction.getValue() / 2.0;

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSwitchInterface footSwitch = footSwitches.get(robotSide);
         footSwitch.getCenterOfPressure(copActual);
         copDesired.setIncludingFrame(desiredCoPs.get(robotSide));

         if (footForceVector.getZ() > minZForceForCoPControlScaling)
         {
            if (copActual.getX() > xThreshold || copDesired.getX() > xThreshold)
               copInForwardWindows.get(robotSide)[writeIndex] = 1;
            else if (copActual.getX() < -xThreshold || copDesired.getX() < -xThreshold)
               copInForwardWindows.get(robotSide)[writeIndex] = -1;
            else
               copInForwardWindows.get(robotSide)[writeIndex] = 0;

            if (copActual.getY() > yThreshold || copDesired.getY() > yThreshold)
               copInSidewaysWindows.get(robotSide)[writeIndex] = 1;
            else if (copActual.getY() < -yThreshold || copDesired.getY() < -yThreshold)
               copInSidewaysWindows.get(robotSide)[writeIndex] = -1;
            else
               copInSidewaysWindows.get(robotSide)[writeIndex] = 0;

            // Update the glitch filter counters
            if (oldIndex < 0 || filteredCopInForwardWindows.get(robotSide)[oldIndex] != copInForwardWindows.get(robotSide)[writeIndex])
               copForwardGlitchCounters.get(robotSide).increment();
            else
               copForwardGlitchCounters.get(robotSide).set(0);

            if (oldIndex < 0 || filteredCopInSidewaysWindows.get(robotSide)[oldIndex] != copInSidewaysWindows.get(robotSide)[writeIndex])
               copSidewaysGlitchCounters.get(robotSide).increment();
            else
               copSidewaysGlitchCounters.get(robotSide).set(0);

            // Update the glitch filter windows
            if (copForwardGlitchCounters.get(robotSide).getValue() >= glitchFilterSize || oldIndex < 0)
               filteredCopInForwardWindows.get(robotSide)[writeIndex] = copInForwardWindows.get(robotSide)[writeIndex];
            else
               filteredCopInForwardWindows.get(robotSide)[writeIndex] = filteredCopInForwardWindows.get(robotSide)[oldIndex];
            if (copSidewaysGlitchCounters.get(robotSide).getValue() >= glitchFilterSize || oldIndex < 0)
               filteredCopInSidewaysWindows.get(robotSide)[writeIndex] = copInSidewaysWindows.get(robotSide)[writeIndex];
            else
               filteredCopInSidewaysWindows.get(robotSide)[writeIndex] = filteredCopInSidewaysWindows.get(robotSide)[oldIndex];
         }
         else
         { // We're not in contact, so set everything to the middle.
            copInForwardWindows.get(robotSide)[writeIndex] = 0;
            copInSidewaysWindows.get(robotSide)[writeIndex] = 0;

            copForwardGlitchCounters.get(robotSide).set(0);
            copSidewaysGlitchCounters.get(robotSide).set(0);

            filteredCopInForwardWindows.get(robotSide)[writeIndex] = 0;
            filteredCopInSidewaysWindows.get(robotSide)[writeIndex] = 0;
         }

         // Update a data holder with the value.
         copForwardSegments.get(robotSide).set(filteredCopInForwardWindows.get(robotSide)[writeIndex]);
         copSidewaysSegments.get(robotSide).set(filteredCopInSidewaysWindows.get(robotSide)[writeIndex]);
      }

      // Update the size of the window to reflect the change.
      windowCurrentSize = Math.min(windowMaxSize, windowCurrentSize + 1);

      // Count the number of times it switches sides
      for (RobotSide robotSide : RobotSide.values)
      {
         int[] forwardWindows = filteredCopInForwardWindows.get(robotSide);
         int[] sidewaysWindows = filteredCopInSidewaysWindows.get(robotSide);

         int currentIndex = getStartOfWindowIndex();

         int copForwardSwitches = 0;
         int copSidewaysSwitches = 0;
         int currentForwardStatus = forwardWindows[currentIndex];
         int currentSidewaysStatus = sidewaysWindows[currentIndex];

         while (currentIndex != writeIndex)
         {
            currentIndex = nextWindowIndex(currentIndex);

            int candidateForwardStatus = forwardWindows[currentIndex];
            int candidateSidewaysStatus = sidewaysWindows[currentIndex];

            if (candidateForwardStatus != currentForwardStatus)
            {  // the zone where the cop has changed. We should assess that.
               if (currentForwardStatus == 0)
               {
                  // If the current status is 0, override it directly, but don't increment. This means basically accepting the first status
                  // of the front or back of the foot.
                  currentForwardStatus = candidateForwardStatus;
               }
               else if (candidateForwardStatus != 0)
               {  // Only enter this if the current status is not in the middle, the candidate status is not in the middle, and they don't match. That means
                  // they've switched.
                  copForwardSwitches++;
                  currentForwardStatus = candidateForwardStatus;
               }
            }
            if (candidateSidewaysStatus != currentSidewaysStatus)
            {  // the zone where the cop has changed. We should assess that.
               if (currentSidewaysStatus == 0)
               {
                  // If the current status is 0, override it directly, but don't increment. This means basically accepting the first status
                  // of the left or right of the foot.
                  currentSidewaysStatus = candidateSidewaysStatus;
               }
               else if (candidateSidewaysStatus != 0)
               {  // Only enter this if the current status is not in the middle, the candidate status is not in the middle, and they don't match. That means
                  // they've switched.
                  copForwardSwitches++;
                  currentSidewaysStatus = candidateSidewaysStatus;
               }
            }
         }

         this.copForwardSwitches.get(robotSide).set(copForwardSwitches);
         this.copSidewaysSwitches.get(robotSide).set(copSidewaysSwitches);
      }
   }

   private int nextWindowIndex(int currentIndex)
   {
      if (currentIndex == windowMaxSize - 1)
         return 0;
      return currentIndex + 1;
   }

   private int getStartOfWindowIndex()
   {
      if (windowCurrentSize == windowMaxSize)
      {
         // the buffer is full, so get the next write index, as it's the start
         return nextWindowIndex(writeIndex);
      }
      else
      {
         // the buffer isn't full, so get the first field
         return 0;
      }
   }
}
