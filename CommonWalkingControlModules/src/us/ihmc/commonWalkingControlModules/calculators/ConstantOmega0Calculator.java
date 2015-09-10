package us.ihmc.commonWalkingControlModules.calculators;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SpatialForceVector;


public class ConstantOmega0Calculator implements Omega0CalculatorInterface
{
   private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable constantOmega0 = new DoubleYoVariable("constantOmega0", registry );
   
   public ConstantOmega0Calculator(double constantOmega0, YoVariableRegistry parentRegistry)
   {
      this.constantOmega0.set(constantOmega0);
      
      parentRegistry.addChild(registry);
   }
   
   public double computeOmega0(SideDependentList<FramePoint2d> cop2ds, SpatialForceVector totalGroundReactionWrench)
   {
      return constantOmega0.getDoubleValue();
   }

}
