package us.ihmc.commonWalkingControlModules.trajectories;

import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;

public class SwingTimeCalculatorProvider implements DoubleProvider
{
   private final SwingTimeCalculator swingTimeCalculator;

   public SwingTimeCalculatorProvider(SwingTimeCalculator swingTimeCalculator)
   {
      this.swingTimeCalculator = swingTimeCalculator;
   }
   
   public double getValue()
   {
      return swingTimeCalculator.getSwingTime();
   }
   
   public SwingTimeCalculator getSwingTimeCalculator()
   {
      return swingTimeCalculator;
   }
   
}
