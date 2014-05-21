package us.ihmc.commonWalkingControlModules.trajectories;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;

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
   
   public double getCurrentSwingTimeValue()
   {
      return transferTime.getDoubleValue();
   }
   
   public void setTransferTime()
   {
      transferTime.set(transferTimeCalculator.getTransferTime());
   }

}
