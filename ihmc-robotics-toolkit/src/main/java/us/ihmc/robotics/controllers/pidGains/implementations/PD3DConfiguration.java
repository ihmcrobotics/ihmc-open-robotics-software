package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PD3DStiffnesses;
import us.ihmc.robotics.controllers.pidGains.PD3DStiffnessesReadOnly;

/**
 * A class that defines the configuration of 3D PID gains. It provides
 * a structure for information needed when creating yo-variable or parameter
 * versions of PID 3D gains.
 */
public class PD3DConfiguration
{

   private final GainCoupling gainCoupling;

   private final PD3DStiffnessesReadOnly gains;

   public PD3DConfiguration(GainCoupling gainCoupling)
   {
      this(gainCoupling, null);
   }

   public PD3DConfiguration(GainCoupling gainCoupling, PD3DStiffnessesReadOnly gains)
   {
      this.gainCoupling = gainCoupling;
      this.gains = gains;
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
   public PD3DStiffnessesReadOnly getGains()
   {
      return gains;
   }
}
