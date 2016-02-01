package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;


public class SwingTimeCalculationProvider implements DoubleProvider
{
//   private int instanceNumber = 0;
   private final ConstantSwingTimeCalculator swingTimeCalculator;
   private final DoubleYoVariable swingTime;

   public SwingTimeCalculationProvider(String name, YoVariableRegistry parentRegistry, ConstantSwingTimeCalculator swingTimeCalculator, double defaultSwingTime)
   {
      YoVariableRegistry registry = new YoVariableRegistry("swingTimeCalculationProvider");
      parentRegistry.addChild(registry);
      this.swingTime = new DoubleYoVariable(name, registry);
      this.swingTimeCalculator = swingTimeCalculator;
      this.swingTime.set(defaultSwingTime);
   }
   
   public double getValue()
   {
      return swingTime.getDoubleValue();
   }
   
   public void updateSwingTime()
   {
      swingTime.set(swingTimeCalculator.getSwingTime());
   }

   public void setSwingTime(double time)
   {
      swingTime.set(time);
   }
}