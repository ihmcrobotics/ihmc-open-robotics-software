package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ParameterizedPIDSE3Gains implements PIDSE3GainsReadOnly
{
   private final ParameterizedPID3DGains positionGains;
   private final ParameterizedPID3DGains orientationGains;
   
   /**
    * Will use the two specified gains internally and
    * avoid creating new gains. <b>This is not a copy constructor.</b> It is
    * useful if this class is used as a wrapper.
    *
    * @param positionGains the gains for the position control.
    * @param orientationGains the gains for the orientation control.
    */
   public ParameterizedPIDSE3Gains(ParameterizedPID3DGains positionGains, ParameterizedPID3DGains orientationGains)
   {
      this.positionGains = positionGains;
      this.orientationGains = orientationGains;
   }
   
   /**
    * Will create a new set of gains will the specified coupling and integration setting for
    * position and orientation.
    *
    * @param suffix the name of the gains will be attached to all parameters.
    * @param gainCouplingPosition the gain coupling for the position gains.
    * @param gainCouplingOrientation the gain coupling for the orientation gains.
    * @param useIntegratorPosition whether the position gains will use an integrator.
    * @param useIntegratorOrientation whether the orientation gains will use an integrator.
    * @param registry the registry to which the tuning variables are attached.
    */
   public ParameterizedPIDSE3Gains(String suffix, GainCoupling gainCouplingPosition, GainCoupling gainCouplingOrientation, boolean useIntegratorPosition,
                                   boolean useIntegratorOrientation, YoRegistry registry)
   {
      this(suffix, gainCouplingPosition, gainCouplingOrientation, useIntegratorPosition, useIntegratorOrientation, null, null, registry);
   }

   /**
    * Will create a new set of gain parameters according to the provided configuration.
    *
    * @param suffix the name of the gains will be attached to all parameters.
    * @param configuration and default values for the gains.
    * @param registry the registry to which the tuning variables are attached.
    */
   public ParameterizedPIDSE3Gains(String suffix, PIDSE3Configuration configuration, YoRegistry registry)
   {
      this(suffix, configuration.getPositionConfiguration(), configuration.getOrientationConfiguration(), registry);
   }

   /**
    * Will create a new set of gain parameters according to the provided configurations for position and orientation
    * gains.
    *
    * @param suffix the name of the gains will be attached to all parameters.
    * @param positionConfiguration configuration and default values for the position part of these gains.
    * @param orientationConfiguration configuration and default values for the orientation part of these gains.
    * @param registry the registry to which the tuning variables are attached.
    */
   public ParameterizedPIDSE3Gains(String suffix, PID3DConfiguration positionConfiguration, PID3DConfiguration orientationConfiguration,
                                   YoRegistry registry)
   {
      this(suffix, positionConfiguration.getGainCoupling(), orientationConfiguration.getGainCoupling(), positionConfiguration.isUseIntegrator(),
           orientationConfiguration.isUseIntegrator(), positionConfiguration.getGains(), orientationConfiguration.getGains(), registry);
   }

   /**
    * Will create a new set of gains will the specified coupling and integration setting for
    * position and orientation. Will use the provided gains as default values for the parameters.
    *
    * @param suffix the name of the gains will be attached to all parameters.
    * @param gainCouplingPosition the gain coupling for the position gains.
    * @param gainCouplingOrientation the gain coupling for the orientation gains.
    * @param useIntegratorPosition whether the position gains will use an integrator.
    * @param useIntegratorOrientation whether the orientation gains will use an integrator.
    * @param defaultPositionGains the default values for the position gains.
    * @param defaultOrientationGains the default values for the orientation gains.
    * @param registry the registry to which the tuning variables are attached.
    */
   public ParameterizedPIDSE3Gains(String suffix, GainCoupling gainCouplingPosition, GainCoupling gainCouplingOrientation, boolean useIntegratorPosition,
                                   boolean useIntegratorOrientation, PID3DGainsReadOnly defaultPositionGains, PID3DGainsReadOnly defaultOrientationGains,
                                   YoRegistry registry)
   {
      positionGains = new ParameterizedPID3DGains(suffix + "Position", gainCouplingPosition, useIntegratorPosition, defaultPositionGains, registry);
      orientationGains = new ParameterizedPID3DGains(suffix + "Orientation", gainCouplingOrientation, useIntegratorOrientation, defaultOrientationGains,
                                                     registry);
   }

   @Override
   public PID3DGainsReadOnly getPositionGains()
   {
      return positionGains;
   }

   @Override
   public PID3DGainsReadOnly getOrientationGains()
   {
      return orientationGains;
   }
   
   

}
