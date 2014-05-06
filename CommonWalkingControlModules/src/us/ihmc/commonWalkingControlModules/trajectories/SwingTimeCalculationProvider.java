package us.ihmc.commonWalkingControlModules.trajectories;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;

public class SwingTimeCalculationProvider implements DoubleProvider
{
   private final SwingTimeCalculator swingTimeCalculator;
   private final DoubleYoVariable swingTime;
   private static int instanceNumber = 0;

   public SwingTimeCalculationProvider(String name, YoVariableRegistry parentRegistry, SwingTimeCalculator swingTimeCalculator, double defaultSwingTime)
   {
      YoVariableRegistry registry = new YoVariableRegistry("swingTimeCalculationProvider"+instanceNumber++);
      parentRegistry.addChild(registry);
      this.swingTime = new DoubleYoVariable(name, registry);
      this.swingTimeCalculator = swingTimeCalculator;
      this.swingTime.set(defaultSwingTime);
   }
   
   public double getValue()
   {
      return swingTime.getDoubleValue();
   }
   
   public void setSwingTimeByDistance(double stepLength)
   {
      swingTime.set(swingTimeCalculator.getSwingTime(stepLength));
   }

   public void setSwingTime(double time)
   {
      swingTime.set(time);
   }
}