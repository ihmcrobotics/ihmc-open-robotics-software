package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SymmetricParameterizedPIDSE3Gains extends ParameterizedPID3DGains implements PIDSE3GainsReadOnly
{
   public SymmetricParameterizedPIDSE3Gains(String suffix, boolean useIntegrator, YoVariableRegistry registry)
   {
      super(suffix, GainCoupling.XYZ, useIntegrator, registry);
   }

   @Override
   public PID3DGainsReadOnly getPositionGains()
   {
      return this;
   }

   @Override
   public PID3DGainsReadOnly getOrientationGains()
   {
      return this;
   }

}
