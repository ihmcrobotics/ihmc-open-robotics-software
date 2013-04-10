package us.ihmc.commonWalkingControlModules.trajectories;

import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;

public class SwingTimeCalculationProvider implements DoubleProvider
{
   private final SwingTimeCalculator swingTimeCalculator;

   public SwingTimeCalculationProvider(SwingTimeCalculator swingTimeCalculator)
   {
      this.swingTimeCalculator = swingTimeCalculator;
   }
   
   public double getValue()
   {
      return swingTimeCalculator.getSwingTime();
   }
   
   public void setSwingTime(double stepLength)
   {
      swingTimeCalculator.setSwingTime(stepLength);
   }
   
}
