package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.YoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SymmetricYoPIDSE3Gains extends DefaultYoPID3DGains implements YoPIDSE3Gains
{
   public SymmetricYoPIDSE3Gains(String suffix, YoVariableRegistry registry)
   {
      super(suffix, GainCoupling.XYZ, true, registry);
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
