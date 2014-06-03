package us.ihmc.valkyrie.kinematics;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class YoDesiredValkyrieJoint implements ValkyrieJointInterface
{
   private final String name;
   private double q;
   private double qd;
   private double f;
   
   private final DoubleYoVariable q_d;
   private final DoubleYoVariable qd_d;
   private final DoubleYoVariable f_d;
   
   public YoDesiredValkyrieJoint(String name, YoVariableRegistry registry)
   {
      this.name = name;
      
      this.q_d = new DoubleYoVariable(name + "_q_d", registry);
      this.qd_d = new DoubleYoVariable(name + "_qd_d", registry);
      this.f_d = new DoubleYoVariable(name + "_tau_d", registry);
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
   public double getDesiredVelocity()
   {
      return qd_d.getDoubleValue();
   }

   @Override
   public void setDesiredVelocity(double velocity)
   {
      this.qd_d.set(velocity);
   }

}
