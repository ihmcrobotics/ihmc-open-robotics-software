package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
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

public class FootShakiesEstimator
{
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
   private final double minZForceForCoPControlScaling;

   // Variables computed by this estimator
   private final SideDependentList<YoFrameVector2D> yoCoPError = new SideDependentList<>();
   private final SideDependentList<YoDouble> yoCoPErrorMagnitude = new SideDependentList<YoDouble>(new YoDouble("leftFootCoPErrorMagnitude", registry),
                                                                                                   new YoDouble("rightFootCoPErrorMagnitude", registry));
   private final YoDouble copRateBreakFrequency = new YoDouble("copRateBreakFrequency", registry);
   private final SideDependentList<FilteredFiniteDifferenceYoVariable> yoCoPErrorRate = new SideDependentList<>();

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

      minZForceForCoPControlScaling = 0.20 * totalMass.getValue() * gravityZ;
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
}
