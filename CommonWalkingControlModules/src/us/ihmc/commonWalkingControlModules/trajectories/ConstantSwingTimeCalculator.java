package us.ihmc.commonWalkingControlModules.trajectories;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ConstantSwingTimeCalculator
{
   private final DoubleYoVariable swingTime;
   
   public ConstantSwingTimeCalculator(double swingTime, YoVariableRegistry registry)
   {
      this.swingTime = new DoubleYoVariable("swingTime", registry);
      this.swingTime.set(swingTime);
   }
   
   public double getSwingTime()
   {
      return swingTime.getDoubleValue();
   }
   
   public void setSwingTime(double swingTime)
   {
      this.swingTime.set(swingTime);
   }

}
