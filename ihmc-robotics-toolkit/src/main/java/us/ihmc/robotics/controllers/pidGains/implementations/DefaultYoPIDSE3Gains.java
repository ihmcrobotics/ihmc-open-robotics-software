package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Provides a default implementation for Yo PID gains for a SE3 PID controller.
 * <p>
 * This class uses two {@link YoPID3DGains}, one for position and one for
 * orientation control, internally.
 * </p>
 */
public class DefaultYoPIDSE3Gains implements YoPIDSE3Gains
{
   private final YoPID3DGains positionGains;
   private final YoPID3DGains orientationGains;

   /**
    * Will use the two specified gains internally and
    * avoid creating new gains. <b>This is not a copy constructor.</b> It is
    * useful if this class is used as a wrapper.
    *
    * @param positionGains the gains for the position control.
    * @param orientationGains the gains for the orientation control.
    */
   public DefaultYoPIDSE3Gains(YoPID3DGains positionGains, YoPID3DGains orientationGains)
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
   public DefaultYoPIDSE3Gains(String suffix, PIDSE3Configuration configuration, YoRegistry registry)
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
   public DefaultYoPIDSE3Gains(String suffix, PID3DConfiguration positionConfiguration, PID3DConfiguration orientationConfiguration,
                               YoRegistry registry)
   {
      this(suffix, positionConfiguration.getGainCoupling(), orientationConfiguration.getGainCoupling(), positionConfiguration.isUseIntegrator(),
           orientationConfiguration.isUseIntegrator(), positionConfiguration.getGains(), orientationConfiguration.getGains(), registry);
   }

   /**
    * Will create a new set of gains with the specified coupling and integration setting for
    * both, position and orientation.
    *
    * @param suffix the name of the gains will be attached to all YoVariables.
    * @param gainCoupling the gain coupling for position and orientation gains.
    * @param useIntegrator whether the position and orientation gains will use an integrator.
    * @param registry the registry to which the tuning variables are attached.
    */
   public DefaultYoPIDSE3Gains(String suffix, GainCoupling gainCoupling, boolean useIntegrator, YoRegistry registry)
   {
      this(suffix, gainCoupling, gainCoupling, useIntegrator, useIntegrator, null, null, registry);
   }

   /**
    * Will create a new set of gains with the specified coupling and integration setting for
    * position and orientation. Will set the new gains to the provided initial values.
    *
    * @param suffix the name of the gains will be attached to all YoVariables.
    * @param gainCouplingPosition the gain coupling for the position gains.
    * @param gainCouplingOrientation the gain coupling for the orientation gains.
    * @param useIntegratorPosition whether the position gains will use an integrator.
    * @param useIntegratorOrientation whether the orientation gains will use an integrator.
    * @param positionGains the initial values for the position gains.
    * @param orientationGains the initial values for the orientation gains.
    * @param registry the registry to which the tuning variables are attached.
    */
   public DefaultYoPIDSE3Gains(String suffix, GainCoupling gainCouplingPosition, GainCoupling gainCouplingOrientation, boolean useIntegratorPosition,
                               boolean useIntegratorOrientation, PID3DGainsReadOnly positionGains, PID3DGainsReadOnly orientationGains, YoRegistry registry)
   {
      this.positionGains = new DefaultYoPID3DGains(suffix + "Position", gainCouplingPosition, useIntegratorPosition, positionGains, registry);
      this.orientationGains = new DefaultYoPID3DGains(suffix + "Orientation", gainCouplingOrientation, useIntegratorOrientation, orientationGains, registry);
   }

   @Override
   public YoPID3DGains getPositionGains()
   {
      return positionGains;
   }

   @Override
   public YoPID3DGains getOrientationGains()
   {
      return orientationGains;
   }
}
