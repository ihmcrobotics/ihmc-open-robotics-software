package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.commonWalkingControlModules.controlModules.foot.SupportStateParameters;

public class LoadBearingParameters
{
   /**
    * Enables the normal-force impedance/admittance controller in post-contact load bearing.
    */
   private final YoBoolean useImpedanceControl;

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
    * Desired contact normal force [N] when impedance control is enabled.
    */
   private final DoubleParameter desiredNormalForce;

   /**
    * Admittance gain [m / (s*N)] mapping normal-force error to normal-axis velocity.
    */
   private final DoubleParameter normalForceAdmittanceGain;

   /**
    * Leak rate [1/s] applied to the integrated normal-axis position offset.
    */
   private final DoubleParameter normalForcePositionLeakRate;

   /**
    * Saturation [m] for the normal-axis position offset commanded by the impedance branch.
    */
   private final DoubleParameter maxNormalPositionOffset;

   /**
    * Saturation [m/s] for the normal-axis velocity commanded by the impedance branch.
    */
   private final DoubleParameter maxNormalVelocity;

   /**
    * Inner-loop normal-axis position gain used to track the impedance reference.
    */
   private final DoubleParameter normalPositionTrackingStiffness;

   /**
    * Inner-loop normal-axis damping ratio used to track the impedance reference.
    */
   private final DoubleParameter normalPositionTrackingDampingRatio;

   public LoadBearingParameters(YoRegistry registry)
   {
      useImpedanceControl = new YoBoolean("useImpedanceControl", registry);
      useImpedanceControl.set(false);
      normalForceThresholdForLoaded = new DoubleParameter("handLoadedForceThreshold", registry, 12.0);
      linearSlippingThreshold = new DoubleParameter("loadBearingLinearTrackingSlipThreshold", registry, 0.06);

      holdPositionStiffness = new DoubleParameter("kpXYHandLoadBearingPosition", registry, 100.0);
      holdPositionDampingRatio = new DoubleParameter("zetaXYHandLoadBearingPosition", registry, 0.65);

      handLoadDuration = new DoubleParameter("handLoadDuration", registry, 0.8);
      desiredNormalForce = new DoubleParameter("desiredHandLoadBearingNormalForce", registry, 20.0);
      normalForceAdmittanceGain = new DoubleParameter("handLoadBearingNormalForceAdmittanceGain", registry, 0.002);
      normalForcePositionLeakRate = new DoubleParameter("handLoadBearingNormalForcePositionLeakRate", registry, 4.0);
      maxNormalPositionOffset = new DoubleParameter("handLoadBearingMaxNormalPositionOffset", registry, 0.03);
      maxNormalVelocity = new DoubleParameter("handLoadBearingMaxNormalVelocity", registry, 0.03);
      normalPositionTrackingStiffness = new DoubleParameter("kpZHandLoadBearingPosition", registry, 150.0);
      normalPositionTrackingDampingRatio = new DoubleParameter("zetaZHandLoadBearingPosition", registry, 1.0);
   }

   public BooleanProvider getUseImpedanceControl()
   {
      return useImpedanceControl;
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

   public double getDesiredNormalForce()
   {
      return desiredNormalForce.getValue();
   }

   public double getNormalForceAdmittanceGain()
   {
      return normalForceAdmittanceGain.getValue();
   }

   public double getNormalForcePositionLeakRate()
   {
      return normalForcePositionLeakRate.getValue();
   }

   public double getMaxNormalPositionOffset()
   {
      return maxNormalPositionOffset.getValue();
   }

   public double getMaxNormalVelocity()
   {
      return maxNormalVelocity.getValue();
   }

   public double getNormalPositionTrackingStiffness()
   {
      return normalPositionTrackingStiffness.getValue();
   }

   public double getNormalPositionTrackingDampingRatio()
   {
      return normalPositionTrackingDampingRatio.getValue();
   }
}
