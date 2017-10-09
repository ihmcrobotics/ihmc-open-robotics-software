package us.ihmc.valkyrie.kinematics;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoValkyrieJointWriter implements ValkyrieJointInterface
{
   private final String name;
   private double q;
   private double qd;
   private double f;
   private double qd_validation;
   private double motorCurrent;
   private double commandedMotorCurrent;

   private final YoDouble q_d;
   private final YoDouble qd_d;
   private final YoDouble f_d;

   public YoValkyrieJointWriter(String name, YoVariableRegistry registry)
   {
      this.name = name;

      this.q_d = new YoDouble(name + "_q_d", registry);
      this.qd_d = new YoDouble(name + "_qd_d", registry);
      this.f_d = new YoDouble(name + "_tau_d", registry);

   }

   @Override
   public void setPosition(double q)
   {
      this.q = q;
   }

   @Override
   public void setVelocity(double qd)
   {
      this.qd = qd;
   }

   @Override
   public void setEffort(double effort)
   {
      this.f = effort;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public double getVelocity()
   {
      return qd;
   }

   @Override
   public double getEffort()
   {
      return f;
   }

   @Override
   public double getValidationVelocity()
   {
      return qd_validation;
   }

   @Override
   public double getPosition()
   {
      return q;
   }

   @Override
   public double getDesiredEffort()
   {
      return f_d.getDoubleValue();
   }

   @Override
   public void setDesiredEffort(double effort)
   {
      this.f_d.set(effort);
   }

   @Override
   public double getDesiredPosition()
   {
      return q_d.getDoubleValue();
   }

   @Override
   public void setDesiredPosition(double position)
   {
      this.q_d.set(position);
   }

   @Override
   public void setValidationVelocity(double velocityFromJacobian)
   {
      this.qd_validation = velocityFromJacobian;
   }

   @Override
   public double getDesiredVelocity()
   {
      return qd_d.getDoubleValue();
   }

   @Override
   public void setDesiredVelocity(double velocity)
   {
      this.qd_d.set(velocity);
   }

   @Override
   public double getMotorCurrent()
   {
      return motorCurrent;
   }

   @Override
   public void setMotorCurrent(double motorCurrent)
   {
      this.motorCurrent = motorCurrent;
   }

   @Override
   public double getCommandedMotorCurrent()
   {
      return commandedMotorCurrent;
   }

   @Override
   public void setCommandedMotorCurrent(double commandedMotorCurrent)
   {
      this.commandedMotorCurrent = commandedMotorCurrent;
   }

   public void set(ValkyrieJointInterface valkyrieJoint)
   {
      setPosition(valkyrieJoint.getPosition());
      setVelocity(valkyrieJoint.getVelocity());
      setEffort(valkyrieJoint.getEffort());
      setMotorCurrent(valkyrieJoint.getMotorCurrent());
      setCommandedMotorCurrent(valkyrieJoint.getCommandedMotorCurrent());
   }

}
