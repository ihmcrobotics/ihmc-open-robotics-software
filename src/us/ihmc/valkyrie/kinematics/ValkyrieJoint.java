package us.ihmc.valkyrie.kinematics;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ValkyrieJoint
{
   private final String name;
   private final DoubleYoVariable position;
   private final DoubleYoVariable velocity;
   private final DoubleYoVariable effort;

   private final DoubleYoVariable desiredPosition;
   private final DoubleYoVariable desiredVelocity;
   private final DoubleYoVariable desiredEffort;

   public ValkyrieJoint(String name, YoVariableRegistry registry)
   {
      this.name = name;
      position = new DoubleYoVariable(name + "_q", registry);
      velocity = new DoubleYoVariable(name + "_qd", registry);
      effort = new DoubleYoVariable(name + "_tau", registry);

      desiredPosition = new DoubleYoVariable(name + "_q_d", registry);
      desiredVelocity = new DoubleYoVariable(name + "_qd_d", registry);
      desiredEffort = new DoubleYoVariable(name + "_tau_d", registry);
   }

   public void setPosition(double q)
   {
      this.position.set(q);
   }

   public void setVelocity(double qd)
   {
      this.velocity.set(qd);
   }

   public void setEffort(double effort)
   {
      this.effort.set(effort);
   }
   
   public String getName()
   {
      return name;
   }

   public double getVelocity()
   {
      return velocity.getDoubleValue();
   }

   public double getEffort()
   {
      return effort.getDoubleValue();
   }

   public double getPosition()
   {
      return position.getDoubleValue();
   }

   public double getDesiredEffort()
   {
      return desiredEffort.getDoubleValue();
   }

   public void setDesiredEffort(double effort)
   {
      desiredEffort.set(effort);
   }

   public double getDesiredPosition()
   {
      return desiredPosition.getDoubleValue();
   }

   public void setDesiredPosition(double position)
   {
      desiredPosition.set(position);
   }

   public double getDesiredVelocity()
   {
      return desiredVelocity.getDoubleValue();
   }

   public void setDesiredVelocity(double velocity)
   {
      desiredVelocity.set(velocity);
   }

}
