package us.ihmc.commonWalkingControlModules.calculators;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ConstantOmega0Calculator implements Omega0CalculatorInterface
{
   private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble constantOmega0 = new YoDouble("constantOmega0", registry);

   public ConstantOmega0Calculator(double constantOmega0, YoVariableRegistry parentRegistry)
   {
      this.constantOmega0.set(constantOmega0);

      parentRegistry.addChild(registry);
   }

   public double computeOmega0(SideDependentList<FramePoint2D> cop2ds, SpatialForceReadOnly totalGroundReactionWrench)
   {
      return constantOmega0.getDoubleValue();
   }

}
