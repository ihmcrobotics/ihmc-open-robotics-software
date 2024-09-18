package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PD3DStiffnesses;
import us.ihmc.robotics.controllers.pidGains.PD3DStiffnessesReadOnly;
import us.ihmc.robotics.controllers.pidGains.YoPD3DStiffnesses;
import us.ihmc.robotics.controllers.pidGains.YoPDSE3Stiffnesses;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Provides a default implementation for Yo PID gains for a SE3 PID controller.
 * <p>
 * This class uses two {@link YoPD3DStiffnesses}, one for position and one for
 * orientation control, internally.
 * </p>
 */
public class DefaultYoPDSE3Stiffnesses implements YoPDSE3Stiffnesses
{
   private final YoPD3DStiffnesses positionGains;
   private final YoPD3DStiffnesses orientationGains;

   /**
    * Will use the two specified gains internally and
    * avoid creating new gains. <b>This is not a copy constructor.</b> It is
    * useful if this class is used as a wrapper.
    *
    * @param positionGains the gains for the position control.
    * @param orientationGains the gains for the orientation control.
    */
   public DefaultYoPDSE3Stiffnesses(YoPD3DStiffnesses positionGains, YoPD3DStiffnesses orientationGains)
   {
      this.positionGains = positionGains;
      this.orientationGains = orientationGains;
   }

   /**
    * Will create a new gains according to the provided gain configuration.
    *
    * @param suffix the name of the gains will be attached to all YoVariables.
    * @param configuration and initial values for the gains.
    * @param registry the registry to which the tuning variables are attached.
    */
   public DefaultYoPDSE3Stiffnesses(String suffix, PDSE3Configuration configuration, YoRegistry registry)
   {
      this(suffix, configuration.getPositionConfiguration(), configuration.getOrientationConfiguration(), registry);
   }

   /**
    * Will create a new gains according to the provided gain configurations for position and orientation gains.
    *
    * @param suffix the name of the gains will be attached to all YoVariables.
    * @param positionConfiguration configuration and initial values for the position gains.
    * @param orientationConfiguration configuration and initial values for the orientation gains.
    * @param registry the registry to which the tuning variables are attached.
    */
   public DefaultYoPDSE3Stiffnesses(String suffix, PD3DConfiguration positionConfiguration, PD3DConfiguration orientationConfiguration,
                                    YoRegistry registry)
   {
      this(suffix, positionConfiguration.getGainCoupling(), orientationConfiguration.getGainCoupling(), positionConfiguration.getGains(), orientationConfiguration.getGains(), registry);
   }

   /**
    * Will create a new set of gains with the specified coupling and integration setting for
    * both, position and orientation.
    *
    * @param suffix the name of the gains will be attached to all YoVariables.
    * @param gainCoupling the gain coupling for position and orientation gains.
    * @param registry the registry to which the tuning variables are attached.
    */
   public DefaultYoPDSE3Stiffnesses(String suffix, GainCoupling gainCoupling, YoRegistry registry)
   {
      this(suffix, gainCoupling, gainCoupling, null, null, registry);
   }

   /**
    * Will create a new set of gains with the specified coupling and integration setting for
    * position and orientation. Will set the new gains to the provided initial values.
    *
    * @param suffix the name of the gains will be attached to all YoVariables.
    * @param gainCouplingPosition the gain coupling for the position gains.
    * @param gainCouplingOrientation the gain coupling for the orientation gains.
    * @param positionGains the initial values for the position gains.
    * @param orientationGains the initial values for the orientation gains.
    * @param registry the registry to which the tuning variables are attached.
    */
   public DefaultYoPDSE3Stiffnesses(String suffix, GainCoupling gainCouplingPosition, GainCoupling gainCouplingOrientation, PD3DStiffnessesReadOnly positionGains, PD3DStiffnessesReadOnly orientationGains, YoRegistry registry)
   {
      this.positionGains = new DefaultYoPD3DStiffnesses(suffix + "Position", gainCouplingPosition, positionGains, registry);
      this.orientationGains = new DefaultYoPD3DStiffnesses(suffix + "Orientation", gainCouplingOrientation, orientationGains, registry);
   }

   @Override
   public YoPD3DStiffnesses getPositionStiffnesses()
   {
      return positionGains;
   }

   @Override
   public YoPD3DStiffnesses getOrientationStiffnesses()
   {
      return orientationGains;
   }
}
