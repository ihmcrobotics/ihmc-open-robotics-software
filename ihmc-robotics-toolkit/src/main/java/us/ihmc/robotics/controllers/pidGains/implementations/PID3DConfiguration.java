package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;

/**
 * A class that defines the configuration of 3D PID gains. It provides
 * a structure for information needed when creating yo-variable or parameter
 * versions of PID 3D gains.
 */
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
   * Returns whether the gains created from this configuration should create
   * an I gain or not (if this returns {@code false} the controller will behave
   * as a PD controller).
   * <p>
   * This is especially useful when using the YoVariable implementation of
   * PID 3D gains {@link DefaultYoPID3DGains} is used since is will avoid
   * creating the YoVariables for tuning the integration.
   * </p>
   * @return whether the gains should include integrator gains.
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
    * should use the same controller gains. This is useful when using a
    * YoVariable implementation of the gains {@link DefaultYoPID3DGains} since
    * it will cause the creation of different set of tuning variables.
    * </p>
    * @return the coupling of the controller gains for the three axes.
    */
   public GainCoupling getGainCoupling()
   {
      return gainCoupling;
   }

   /**
    * Returns the default set of gains to be used when PID gains are
    * created from this configuration. This can be null if the configuration
    * was created without default gains.
    *
    * @return default gains.
    */
   public PID3DGainsReadOnly getGains()
   {
      return gains;
   }
}
