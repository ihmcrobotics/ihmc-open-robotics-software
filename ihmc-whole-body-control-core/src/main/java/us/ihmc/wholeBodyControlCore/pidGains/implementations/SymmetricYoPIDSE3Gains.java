package us.ihmc.wholeBodyControlCore.pidGains.implementations;

import us.ihmc.wholeBodyControlCore.pidGains.GainCoupling;
import us.ihmc.wholeBodyControlCore.pidGains.PID3DGainsBasics;
import us.ihmc.wholeBodyControlCore.pidGains.PIDSE3GainsBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Provides symmetric PID gains for SE3 control. This means that all six axes will use
 * the same PID gains.
 */
public class SymmetricYoPIDSE3Gains implements PIDSE3GainsBasics
{
   private final YoPID3DGains gains;

   public SymmetricYoPIDSE3Gains(String suffix, YoRegistry registry)
   {
      this(suffix, true, registry);
   }

   public SymmetricYoPIDSE3Gains(String suffix, boolean useIntegrator, YoRegistry registry)
   {
      gains = new YoPID3DGains(suffix, GainCoupling.XYZ, useIntegrator, registry);
   }

   @Override
   public PID3DGainsBasics getPositionGains()
   {
      return gains;
   }

   @Override
   public PID3DGainsBasics getOrientationGains()
   {
      return gains;
   }

   public void setProportionalGains(double proportionalGains)
   {
      this.gains.setProportionalGains(proportionalGains);
   }

   public void setDampingRatios(double derivativeGains)
   {
      this.gains.setDampingRatios(derivativeGains);
   }

   public void setDerivativeGains(double derivativeGains)
   {
      this.gains.setDerivativeGains(derivativeGains);
   }
}
