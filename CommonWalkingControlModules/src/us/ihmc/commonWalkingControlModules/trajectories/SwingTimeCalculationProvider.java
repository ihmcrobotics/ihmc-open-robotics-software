package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


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