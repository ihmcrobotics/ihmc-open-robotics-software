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
