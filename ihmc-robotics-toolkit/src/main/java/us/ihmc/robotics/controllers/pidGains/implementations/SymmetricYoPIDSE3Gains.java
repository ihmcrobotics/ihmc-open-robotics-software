package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Provides symmetric PID gains for SE3 control. This means that all six axes will use
 * the same PID gains.
 */
public class SymmetricYoPIDSE3Gains extends DefaultYoPID3DGains implements YoPIDSE3Gains
{
   public SymmetricYoPIDSE3Gains(String suffix, YoVariableRegistry registry)
   {
      super(suffix, GainCoupling.XYZ, true, registry);
   }

   public SymmetricYoPIDSE3Gains(String suffix, boolean useIntegrator, YoVariableRegistry registry)
   {
      super(suffix, GainCoupling.XYZ, useIntegrator, registry);
   }

   @Override
   public YoPID3DGains getPositionGains()
   {
      return this;
   }

   @Override
   public YoPID3DGains getOrientationGains()
   {
      return this;
   }
}
