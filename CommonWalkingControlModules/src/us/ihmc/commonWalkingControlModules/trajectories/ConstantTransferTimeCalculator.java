package us.ihmc.commonWalkingControlModules.trajectories;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ConstantTransferTimeCalculator 
{
   private final DoubleYoVariable transferTime;
   
   public ConstantTransferTimeCalculator(double transferTime, YoVariableRegistry registry)
   {
      this.transferTime = new DoubleYoVariable("transferTime", registry);
      this.transferTime.set(transferTime);
   }
   
   public double getTransferTime()
   {
      return transferTime.getDoubleValue();
   }
   
   public void setTransferTime(double transferTime)
   {
      this.transferTime.set(transferTime);
   }

}

