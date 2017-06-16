package us.ihmc.steppr.hardware.state;

import java.nio.ByteBuffer;

import us.ihmc.acsell.hardware.state.AcsellPowerDistributionADCState;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.steppr.parameters.BonoRobotModel;

public class StepprPowerDistributionADCState implements AcsellPowerDistributionADCState
{
   private final YoVariableRegistry registry;
   private final YoInteger ADC[] = new YoInteger[8];

   private final YoDouble robotPower;
   private final YoDouble robotWork;
   private final YoDouble busVoltage;
   private final YoDouble leftLimbCurrent;
   private final YoDouble rightLimbCurrent;
   private final YoDouble torsoLimbCurrent;
   private final double dt;

   public StepprPowerDistributionADCState(String name, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(name);
      for (int i = 0; i < ADC.length; i++)
      {
         ADC[i] = new YoInteger("rawADC" + i, registry);
      }

      robotPower = new YoDouble("robotPower", registry);
      robotWork = new YoDouble("robotWork", registry);
      busVoltage = new YoDouble("busVoltage", registry);
      leftLimbCurrent = new YoDouble("leftLimbCurrent", registry);
      rightLimbCurrent = new YoDouble("rightLimbCurrent", registry);
      torsoLimbCurrent = new YoDouble("torsoLimbCurrent", registry);
      dt = (new BonoRobotModel(true, false)).getEstimatorDT();
      parentRegistry.addChild(registry);
   }

   public void update(ByteBuffer buffer)
   {
      for (int i = 0; i < ADC.length; i++)
      {
         ADC[i].set(buffer.getShort());
      }

      busVoltage.set(((double) (ADC[0].getIntegerValue() & 0xFFFF)) / 491.0 - 0.1);
      leftLimbCurrent.set((ADC[1].getValueAsDouble() + 16.0) * 0.0061);
      rightLimbCurrent.set((ADC[2].getValueAsDouble() + 14.0) * 0.0061);
      torsoLimbCurrent.set((ADC[3].getValueAsDouble() + 15.0) * 0.0061);

      robotPower.set(busVoltage.getDoubleValue() * (leftLimbCurrent.getDoubleValue() + rightLimbCurrent.getDoubleValue() + torsoLimbCurrent.getDoubleValue()));
      robotWork.add(robotPower.getDoubleValue() *  dt);
   }

   @Override
   public YoDouble getTotalWorkVariable()
   {
      return robotWork;
   }
}
