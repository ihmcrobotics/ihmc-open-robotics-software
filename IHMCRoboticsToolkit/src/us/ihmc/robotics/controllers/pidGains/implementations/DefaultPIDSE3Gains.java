package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;

/**
 * Provides a default implementation for PID gains for a SE3 PID controller.
 * <p>
 * This class uses two {@link DefaultPID3DGains}, one for position and one for
 * orientation control, internally.
 * </p>
 */
public class DefaultPIDSE3Gains implements PIDSE3Gains
{
   private final DefaultPID3DGains positionGains;
   private final DefaultPID3DGains orientationGains;

   /**
    * Creates new gains with no gain coupling and integrator gains.
    */
   public DefaultPIDSE3Gains()
   {
      this(GainCoupling.NONE, true);
   }

   /**
    * Creates new gains with the specifies settings.
    *
    * @param gainCoupling the gain coupling for position and orientation gains.
    * @param useIntegrator whether the position and orientation gains will use an integrator.
    */
   public DefaultPIDSE3Gains(GainCoupling gainCoupling, boolean useIntegrator)
   {
      this(gainCoupling, gainCoupling, useIntegrator, useIntegrator);
   }

   /**
    * Creates new gains with the specifies settings.
    *
    * @param gainCouplingPosition the gain coupling for the position gains.
    * @param gainCouplingOrientation the gain coupling for the orientation gains.
    * @param useIntegratorPosition whether the position gains will use an integrator.
    * @param useIntegratorOrientation whether the orientation gains will use an integrator.
    */
   public DefaultPIDSE3Gains(GainCoupling gainCouplingPosition, GainCoupling gainCouplingOrientation, boolean useIntegratorPosition,
                             boolean useIntegratorOrientation)
   {
      positionGains = new DefaultPID3DGains(gainCouplingPosition, useIntegratorPosition);
      orientationGains = new DefaultPID3DGains(gainCouplingOrientation, useIntegratorOrientation);
   }

   @Override
   public DefaultPID3DGains getPositionGains()
   {
      return positionGains;
   }

   @Override
   public DefaultPID3DGains getOrientationGains()
   {
      return orientationGains;
   }

   public void setPositionDampingRatios(double dampingRatioX, double dampingRatioY, double dampingRatioZ)
   {
      getPositionGains().setDampingRatios(dampingRatioX, dampingRatioY, dampingRatioZ);
   }

   public void setOrientationDampingRatios(double dampingRatioX, double dampingRatioY, double dampingRatioZ)
   {
      getOrientationGains().setDampingRatios(dampingRatioX, dampingRatioY, dampingRatioZ);
   }

   public void setPositionDampingRatios(double dampingRatio)
   {
      getPositionGains().setDampingRatios(dampingRatio);
   }

   public void setOrientationDampingRatios(double dampingRatio)
   {
      getOrientationGains().setDampingRatios(dampingRatio);
   }
}
