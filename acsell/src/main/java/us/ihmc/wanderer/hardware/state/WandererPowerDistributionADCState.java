package us.ihmc.wanderer.hardware.state;

import java.nio.ByteBuffer;

import us.ihmc.acsell.hardware.state.AcsellPowerDistributionADCState;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.wanderer.parameters.WandererRobotModel;

public class WandererPowerDistributionADCState implements AcsellPowerDistributionADCState
{
   private final YoVariableRegistry registry;
   private final YoInteger ADC[] = new YoInteger[8];

   private final YoDouble robotPower;
   private final YoDouble robotWork;
   private final YoDouble busVoltage;
   private final YoDouble leftLimbCurrent;
   private final YoDouble rightLimbCurrent;
   private final YoDouble torsoLimbCurrent;
   private final YoDouble vicor48Current;
   private final YoDouble vicor12Current;
   private final YoDouble armCurrent;
   private final YoDouble inputCurrent;
   private final AlphaFilteredYoVariable averageRobotPower;
   private final double dt;

   public WandererPowerDistributionADCState(String name, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(name);
      for (int i = 0; i < ADC.length; i++)
      {
         ADC[i] = new YoInteger("rawADC" + i, registry);
      }

      robotPower = new YoDouble("robotPower", registry);
      averageRobotPower = new AlphaFilteredYoVariable("averageRobotPower", registry, 0.99975, robotPower);
      robotWork = new YoDouble("robotWork", registry);
      busVoltage = new YoDouble("busVoltage", registry);
      leftLimbCurrent = new YoDouble("leftLimbCurrent", registry);
      rightLimbCurrent = new YoDouble("rightLimbCurrent", registry);
      torsoLimbCurrent = new YoDouble("torsoLimbCurrent", registry);
      vicor48Current = new YoDouble("vicor48Current", registry);
      vicor12Current = new YoDouble("vicor12Current", registry);
      armCurrent = new YoDouble("armCurrent", registry);
      inputCurrent = new YoDouble("inputCurrent", registry);
      
      dt = (new WandererRobotModel(true, false)).getEstimatorDT();
      parentRegistry.addChild(registry);
   }

   public void update(ByteBuffer buffer)
   {
      for (int i = 0; i < ADC.length; i++)
      {
         ADC[i].set(buffer.getShort());
      }

      busVoltage.set(((double) (ADC[6].getIntegerValue() & 0xFFFF)) / 68.7 - 0.1);
      armCurrent.set(-(ADC[0].getValueAsDouble() + 0.5) * 0.0244);
      vicor48Current.set(-(ADC[1].getValueAsDouble() - 1.5) * 0.0244);
      vicor12Current.set(-(ADC[2].getValueAsDouble() + 0.3) * 0.0244);
      torsoLimbCurrent.set(-(ADC[3].getValueAsDouble() - 2.0) * 0.0244);
      leftLimbCurrent.set(-(ADC[4].getValueAsDouble() + 0.0) * 0.0244);
      rightLimbCurrent.set(-(ADC[5].getValueAsDouble() + 0.0) * 0.0244);
      inputCurrent.set((ADC[7].getValueAsDouble() + 0.0) * 0.0244);

      robotPower.set(busVoltage.getDoubleValue() * (inputCurrent.getDoubleValue()));
      robotWork.add(robotPower.getDoubleValue() *  dt);
      averageRobotPower.update();
   }

   @Override
   public YoDouble getTotalWorkVariable()
   {
      return robotWork;
   }
}
