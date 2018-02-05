package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;

public class PID3DConfiguration
{
   private final boolean useIntegrator;

   private final GainCoupling gainCoupling;

   private final PID3DGainsReadOnly gains;

   public PID3DConfiguration(GainCoupling gainCoupling, boolean useIntegrator)
   {
      this(gainCoupling, useIntegrator, null);
   }

   public PID3DConfiguration(GainCoupling gainCoupling, boolean useIntegrator, PID3DGainsReadOnly gains)
   {
      this.useIntegrator = useIntegrator;
      this.gainCoupling = gainCoupling;
      this.gains = gains;
   }

   /**
   * Returns whether the PID controller used an I gain or not (if this
   * returns {@code false} the controller will behave as a PD controller).
   * <p>
   * This is especially useful when using the YoVariable implementation of
   * this class {@link DefaultYoPID3DGains} is used since is will avoid
   * creating the YoVariables for tuning the integration.
   * </p>
   * @return whether the gains include integrator gains.
   */
   public boolean isUseIntegrator()
   {
      return useIntegrator;
   }

   /**
    * Returns the type of gain coupling for the three dimensions.
    * <p>
    * The gain coupling determines whether all axis should use different
    * gains or if some axis (e.g. the X and Y axis for {@link GainCoupling#XY})
    * should use the same controller gains. This is useful when using the
    * YoVariable implementation of this class {@link DefaultYoPID3DGains} since
    * it will cause the creation of different set of tuning variables.
    * </p>
    * @return the coupling of the controller gains for the three axes.
    */
   public GainCoupling getGainCoupling()
   {
      return gainCoupling;
   }

   public boolean hasGains()
   {
      return gains != null;
   }

   public PID3DGainsReadOnly getGains()
   {
      return gains;
   }
}
