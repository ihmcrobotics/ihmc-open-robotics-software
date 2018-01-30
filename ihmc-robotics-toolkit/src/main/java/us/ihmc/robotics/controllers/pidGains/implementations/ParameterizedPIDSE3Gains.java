package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

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
                                   boolean useIntegratorOrientation, YoVariableRegistry registry)
   {
      positionGains = new ParameterizedPID3DGains(suffix + "Position", gainCouplingPosition, useIntegratorPosition, registry);
      orientationGains = new ParameterizedPID3DGains(suffix + "Orientation", gainCouplingOrientation, useIntegratorOrientation, registry);
   }

   /**
    * Will create a new set of gain parameters. The provided gain values are used as default
    * values for the parameters.
    *
    * @param suffix the name of the gains will be attached to all parameters.
    * @param defaultGains default values for the gains.
    * @param registry the registry to which the tuning variables are attached.
    */
   public ParameterizedPIDSE3Gains(String suffix, PIDSE3GainsReadOnly defaultGains, YoVariableRegistry registry)
   {
      this(suffix, defaultGains.getPositionGains(), defaultGains.getOrientationGains(), registry);
   }

   /**
    * Will create a new set of gain parameters. The provided values for position and orientation
    * gains are used as default values for the parameters.
    *
    * @param suffix the name of the gains will be attached to all parameters.
    * @param defaultPositionGains default values for the position part of these gains.
    * @param defaultOrientationGains default values for the orientation part of these gains.
    * @param registry the registry to which the tuning variables are attached.
    */
   public ParameterizedPIDSE3Gains(String suffix, PID3DGainsReadOnly defaultPositionGains, PID3DGainsReadOnly defaultOrientationGains,
                                   YoVariableRegistry registry)
   {
      positionGains = new ParameterizedPID3DGains(suffix, defaultPositionGains, registry);
      orientationGains = new ParameterizedPID3DGains(suffix, defaultOrientationGains, registry);
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
