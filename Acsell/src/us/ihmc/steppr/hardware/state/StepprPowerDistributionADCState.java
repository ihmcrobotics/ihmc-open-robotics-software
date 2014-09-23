package us.ihmc.steppr.hardware.state;

import java.nio.ByteBuffer;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;

public class StepprPowerDistributionADCState
{
   private final YoVariableRegistry registry;
   private final IntegerYoVariable ADC[] = new IntegerYoVariable[8];
   
   public StepprPowerDistributionADCState(String name, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(name);
      for(int i = 0; i < ADC.length; i++)
      {
         ADC[i] = new IntegerYoVariable("ADC" + i, registry);
      }
      
      parentRegistry.addChild(registry);
   }
   
   public void update(ByteBuffer buffer)
   {
      for(int i = 0; i < ADC.length; i++)
      {
         ADC[i].set(buffer.getShort());
      }
   }
}
