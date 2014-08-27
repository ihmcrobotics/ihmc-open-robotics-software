package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


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

