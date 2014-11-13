package us.ihmc.steppr.hardware.state;

import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.utilities.math.geometry.AngleTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class StepprKneeJointState implements StepprJointState
{
   private final YoVariableRegistry registry;

   private final StepprActuatorState actuator;
   private final StepprActuatorState ankle;

   private final double ratio;

   private double motorAngle;
   
   private final DoubleYoVariable q;
   private final DoubleYoVariable qd;
   private final DoubleYoVariable tau;

   public StepprKneeJointState(StepprJoint joint, StepprActuatorState actuator, StepprActuatorState ankle, YoVariableRegistry parentRegistry)
   {
      String name = joint.getSdfName();
      this.registry = new YoVariableRegistry(name);
      this.ratio = joint.getRatio();
      this.actuator = actuator;
      this.ankle = ankle;
      
      this.q = new DoubleYoVariable(name + "_q", registry);
      this.qd = new DoubleYoVariable(name + "_qd", registry);
      this.tau = new DoubleYoVariable(name + "_tau", registry);
      
      parentRegistry.addChild(registry);
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
      
      motorAngle = actuator.getMotorPosition();
      
      double ankleAngle = ankle.getMotorPosition();
      double ankleVelocity = ankle.getMotorVelocity();
      
      q.set(AngleTools.trimAngleMinusPiToPi(actuator.getJointPosition() + ankleAngle));
      qd.set(actuator.getJointVelocity() + ankleVelocity);
      tau.set(actuator.getMotorTorque() * ratio);
   }

   @Override
   public int getNumberOfActuators()
   {
      return 1;
   }

   @Override
   public double getMotorAngle(int actuator)
   {
      return motorAngle;
   }

}
