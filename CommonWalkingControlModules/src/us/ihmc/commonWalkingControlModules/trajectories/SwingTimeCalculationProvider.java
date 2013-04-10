package us.ihmc.commonWalkingControlModules.trajectories;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;

public class SwingTimeCalculationProvider implements DoubleProvider
{
   private final SwingTimeCalculator swingTimeCalculator;
   private final DoubleYoVariable swingTime;

   public SwingTimeCalculationProvider(String name, YoVariableRegistry registry, SwingTimeCalculator swingTimeCalculator, double defaultSwingTime)
   {
      this.swingTime = new DoubleYoVariable(name, registry);
      this.swingTimeCalculator = swingTimeCalculator;
      this.swingTime.set(defaultSwingTime);
   }
   
   public double getValue()
   {
      return swingTime.getDoubleValue();
   }
   
   public void setSwingTime(double stepLength)
   {
      swingTime.set(swingTimeCalculator.getSwingTime(stepLength));
   }

}