package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

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
    * Will create a copy of the provided {@link PIDSE3Gains}
    * in the given registry and using the provided name prefix.
    *
    * @param suffix the name of the gains will be attached to all YoVariables.
    * @param other the gains that will be copied into this object.
    * @param registry the registry to which the tuning variables are attached.
    */
   public DefaultYoPIDSE3Gains(String suffix, PIDSE3Gains other, YoVariableRegistry registry)
   {
      this(suffix, other.getPositionGains(), other.getOrientationGains(), registry);
   }

   /**
    * Will create a copy of the provided {@link PID3DGainsReadOnly}
    * for position and orientation in the given registry and using the provided name prefix.
    *
    * @param suffix the name of the gains will be attached to all YoVariables.
    * @param positionGains the position gains to be copied.
    * @param orientationGains the orientation gains to be copied.
    * @param registry the registry to which the tuning variables are attached.
    */
   public DefaultYoPIDSE3Gains(String suffix, PID3DGainsReadOnly positionGains, PID3DGainsReadOnly orientationGains, YoVariableRegistry registry)
   {
      this(suffix, positionGains.getGainCoupling(), orientationGains.getGainCoupling(), positionGains.isUseIntegrator(), orientationGains.isUseIntegrator(),
           registry);
      setPositionGains(positionGains);
      setOrientationGains(orientationGains);
   }

   /**
    * Will create a new set of gains will the specified coupling and integration setting for
    * both, position and orientation.
    *
    * @param suffix the name of the gains will be attached to all YoVariables.
    * @param gainCoupling the gain coupling for position and orientation gains.
    * @param useIntegrator whether the position and orientation gains will use an integrator.
    * @param registry the registry to which the tuning variables are attached.
    */
   public DefaultYoPIDSE3Gains(String suffix, GainCoupling gainCoupling, boolean useIntegrator, YoVariableRegistry registry)
   {
      this(suffix, gainCoupling, gainCoupling, useIntegrator, useIntegrator, registry);
   }

   /**
    * Will create a new set of gains will the specified coupling and integration setting for
    * position and orientation.
    *
    * @param suffix the name of the gains will be attached to all YoVariables.
    * @param gainCouplingPosition the gain coupling for the position gains.
    * @param gainCouplingOrientation the gain coupling for the orientation gains.
    * @param useIntegratorPosition whether the position gains will use an integrator.
    * @param useIntegratorOrientation whether the orientation gains will use an integrator.
    * @param registry the registry to which the tuning variables are attached.
    */
   public DefaultYoPIDSE3Gains(String suffix, GainCoupling gainCouplingPosition, GainCoupling gainCouplingOrientation, boolean useIntegratorPosition,
                               boolean useIntegratorOrientation, YoVariableRegistry registry)
   {
      positionGains = new DefaultYoPID3DGains(suffix + "Position", gainCouplingPosition, useIntegratorPosition, registry);
      orientationGains = new DefaultYoPID3DGains(suffix + "Orientation", gainCouplingOrientation, useIntegratorOrientation, registry);
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
