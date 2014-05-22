package us.ihmc.commonWalkingControlModules.trajectories;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;

public class SwingTimeCalculationProvider implements DoubleProvider
{
   private static int instanceNumber = 0;
   private final SwingTimeCalculator swingTimeCalculator;
   private final DoubleYoVariable swingTime;
   private final double defaultSwingTime; 
   private double fastSwingTime; 

   public SwingTimeCalculationProvider(String name, YoVariableRegistry parentRegistry, SwingTimeCalculator swingTimeCalculator, double defaultSwingTime)
   {
      YoVariableRegistry registry = new YoVariableRegistry("swingTimeCalculationProvider"+instanceNumber++);
      parentRegistry.addChild(registry);
      this.swingTime = new DoubleYoVariable(name, registry);
      this.swingTimeCalculator = swingTimeCalculator;
      this.swingTime.set(defaultSwingTime);
      this.defaultSwingTime = defaultSwingTime;
      this.fastSwingTime = defaultSwingTime;
   }
   
   public double getValue()
   {
      return swingTime.getDoubleValue();
   }
   
   public void resetFastSwingTime()
   {
      this.fastSwingTime = this.defaultSwingTime;
   }
   
   public void setFastSwingTime(double time)
   {
      this.fastSwingTime = time;
   }
   
   public void setSwingTimeByDistance(double stepLength)
   {
      swingTime.set(swingTimeCalculator.getSwingTime(stepLength));
   }

   public void setSwingTime(double time)
   {
      swingTime.set(time);
   }
   
   public void useDefaultSwingTime()
   {
      this.swingTime.set(this.defaultSwingTime);
   }
   
   public void useFastSwingTime()
   {
      this.swingTime.set(this.fastSwingTime);
   }
}