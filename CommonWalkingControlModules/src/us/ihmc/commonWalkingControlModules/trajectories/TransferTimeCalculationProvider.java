package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


public class TransferTimeCalculationProvider implements DoubleProvider
{
   private final ConstantTransferTimeCalculator transferTimeCalculator;
   private final DoubleYoVariable transferTime;

   public TransferTimeCalculationProvider(String name, YoVariableRegistry registry, ConstantTransferTimeCalculator transferTimeCalculator, double defaultTransferTime)
   {
      this.transferTime = new DoubleYoVariable(name, registry);
      this.transferTimeCalculator = transferTimeCalculator;
      this.transferTime.set(defaultTransferTime);
   }
   
   public double getValue()
   {
      return transferTime.getDoubleValue();
   }
   
   public void updateTransferTime()
   {
      transferTime.set(transferTimeCalculator.getTransferTime());
   }

}
