package us.ihmc.wanderer.hardware.state;

import java.nio.ByteBuffer;

import us.ihmc.acsell.hardware.state.AcsellPowerDistributionADCState;
import us.ihmc.wanderer.parameters.WandererRobotModel;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;

public class WandererPowerDistributionADCState implements AcsellPowerDistributionADCState
{
   private final YoVariableRegistry registry;
   private final IntegerYoVariable ADC[] = new IntegerYoVariable[8];

   private final DoubleYoVariable robotPower;
   private final DoubleYoVariable robotWork;
   private final DoubleYoVariable busVoltage;
   private final DoubleYoVariable leftLimbCurrent;
   private final DoubleYoVariable rightLimbCurrent;
   private final DoubleYoVariable torsoLimbCurrent;
   private final DoubleYoVariable vicor48Current;
   private final DoubleYoVariable vicor12Current;
   private final DoubleYoVariable armCurrent;
   private final DoubleYoVariable inputCurrent;
   private final double dt;

   public WandererPowerDistributionADCState(String name, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(name);
      for (int i = 0; i < ADC.length; i++)
      {
         ADC[i] = new IntegerYoVariable("rawADC" + i, registry);
      }

      robotPower = new DoubleYoVariable("robotPower", registry);
      robotWork = new DoubleYoVariable("robotWork", registry);
      busVoltage = new DoubleYoVariable("busVoltage", registry);
      leftLimbCurrent = new DoubleYoVariable("leftLimbCurrent", registry);
      rightLimbCurrent = new DoubleYoVariable("rightLimbCurrent", registry);
      torsoLimbCurrent = new DoubleYoVariable("torsoLimbCurrent", registry);
      vicor48Current = new DoubleYoVariable("vicor48Current", registry);
      vicor12Current = new DoubleYoVariable("vicor12Current", registry);
      armCurrent = new DoubleYoVariable("armCurrent", registry);
      inputCurrent = new DoubleYoVariable("inputCurrent", registry);
      
      dt = (new WandererRobotModel(true, false)).getEstimatorDT();
      parentRegistry.addChild(registry);
   }

   public void update(ByteBuffer buffer)
   {
      for (int i = 0; i < ADC.length; i++)
      {
         ADC[i].set(buffer.getShort());
      }

      busVoltage.set(((double) (ADC[6].getIntegerValue() & 0xFFFF)) / 491.0 - 0.1);
      leftLimbCurrent.set((ADC[0].getValueAsDouble() + 0.0) * 0.0061);
      rightLimbCurrent.set((ADC[1].getValueAsDouble() + 0.0) * 0.0061);
      torsoLimbCurrent.set((ADC[2].getValueAsDouble() + 0.0) * 0.0061);
      vicor48Current.set((ADC[3].getValueAsDouble() + 0.0) * 0.0061);
      vicor12Current.set((ADC[4].getValueAsDouble() + 0.0) * 0.0061);
      armCurrent.set((ADC[5].getValueAsDouble() + 0.0) * 0.0061);
      inputCurrent.set((ADC[7].getValueAsDouble() + 0.0) * 0.0061);

      robotPower.set(busVoltage.getDoubleValue() * (leftLimbCurrent.getDoubleValue() + rightLimbCurrent.getDoubleValue() + torsoLimbCurrent.getDoubleValue()));
      robotWork.add(robotPower.getDoubleValue() *  dt);
   }
}
