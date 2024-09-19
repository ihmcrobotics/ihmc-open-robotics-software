package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.commonWalkingControlModules.controlModules.foot.SupportStateParameters;

public class LoadBearingParameters
{
   /**
    * Threshold [Nm] below which the hand has a non-slip feedback objective. Analogous to {@link SupportStateParameters#getFootLoadThreshold}
    */
   private final DoubleParameter normalForceThresholdForLoaded;

   /**
    * Tracking error threshold [m], if the hand has a position tracking error greater it will exit load bearing and hold position
    */
   private final DoubleParameter linearSlippingThreshold;

   /**
    * Stiffness of the hold position objective (when barely loaded)
    */
   private final DoubleParameter holdPositionStiffness;

   /**
    * Damping ratio of the hold position objective (when barely loaded)
    */
   private final DoubleParameter holdPositionDampingRatio;

   /**
    * Duration (in seconds) of time taken to load the hand
    */
   private final DoubleParameter handLoadDuration;

   /**
    * Whether to have a smooth loading phase of the hand
    */
   private final BooleanParameter doSmoothHandLoading;

   public LoadBearingParameters(YoRegistry registry)
   {
      normalForceThresholdForLoaded = new DoubleParameter("handLoadedForceThreshold", registry, 16.0);
      linearSlippingThreshold = new DoubleParameter("loadBearingLinearTrackingSlipThreshold", registry, 0.12);

      holdPositionStiffness = new DoubleParameter("kpXYHandLoadBearingPosition", registry, 100.0);
      holdPositionDampingRatio = new DoubleParameter("zetaXYHandLoadBearingPosition", registry, 0.65);

      handLoadDuration = new DoubleParameter("handLoadDuration", registry, 0.5);
      doSmoothHandLoading = new BooleanParameter("doSmoothHandLoading", registry, false);
   }

   public double getNormalForceThresholdForLoaded()
   {
      return normalForceThresholdForLoaded.getValue();
   }

   public double getLinearTrackingSlipThreshold()
   {
      return linearSlippingThreshold.getValue();
   }

   public double getHoldPositionStiffness()
   {
      return holdPositionStiffness.getValue();
   }

   public double getHoldPositionDampingRatio()
   {
      return holdPositionDampingRatio.getValue();
   }

   public double getHandLoadDuration()
   {
      return handLoadDuration.getValue();
   }

   public boolean doSmoothHandLoading()
   {
      return doSmoothHandLoading.getValue();
   }
}
