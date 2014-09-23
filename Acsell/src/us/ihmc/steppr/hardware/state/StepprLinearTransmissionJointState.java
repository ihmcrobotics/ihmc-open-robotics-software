package us.ihmc.steppr.hardware.state;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class StepprLinearTransmissionJointState implements StepprJointState
{
   private final YoVariableRegistry registry;

   private final StepprActuatorState actuator;

   private final double ratio;

   private final DoubleYoVariable q;
   private final DoubleYoVariable qd;
   private final DoubleYoVariable tau;

   public StepprLinearTransmissionJointState(String name, double ratio, StepprActuatorState actuator, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(name);
      this.ratio = ratio;
      this.actuator = actuator;

      this.q = new DoubleYoVariable("q", registry);
      this.qd = new DoubleYoVariable("qd", registry);
      this.tau = new DoubleYoVariable("tau", registry);
   }

   @Override
   public double getQ()
   {
      return q.getDoubleValue();
   }

   @Override
   public double getQd()
   {
      return qd.getDoubleValue();
   }

   @Override
   public double getTau()
   {
      return tau.getDoubleValue();
   }

   @Override
   public void update()
   {
      q.set(actuator.getMotorPosition() / ratio);
      qd.set(actuator.getMotorVelocity() / ratio);
      tau.set(actuator.getMotorTorque() * ratio);
   }

}
