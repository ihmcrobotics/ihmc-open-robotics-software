package us.ihmc.valkyrie.kinematics;

import us.ihmc.valkyrie.ValkyrieControllerFactory;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.filter.FilteredVelocityYoVariable;

public class ValkyrieJoint
{
   private static final double DT = ValkyrieControllerFactory.getEstimatorDT();
   private static final double ALPHA = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(50.0, DT);
   
   private final String name;
   private final DoubleYoVariable position;
   private final DoubleYoVariable velocity;
   private final FilteredVelocityYoVariable fd_velocity;
   private final AlphaFilteredYoVariable filt_velocity;
   private final DoubleYoVariable effort;

   private final DoubleYoVariable desiredPosition;
   private final DoubleYoVariable desiredVelocity;
   private final DoubleYoVariable desiredEffort;

   public ValkyrieJoint(String name, YoVariableRegistry registry)
   {
      this.name = name;
      position = new DoubleYoVariable(name + "_q", registry);
      velocity = new DoubleYoVariable(name + "_qd", registry);
      fd_velocity = new FilteredVelocityYoVariable(name + "_fd_qd", "", ALPHA, position, DT, registry);
      filt_velocity = new AlphaFilteredYoVariable(name + "_filt_qd", registry, ALPHA, fd_velocity);
      effort = new DoubleYoVariable(name + "_tau", registry);

      desiredPosition = new DoubleYoVariable(name + "_q_d", registry);
      desiredVelocity = new DoubleYoVariable(name + "_qd_d", registry);
      desiredEffort = new DoubleYoVariable(name + "_tau_d", registry);
   }

   public void setPosition(double q)
   {
      this.position.set(q);
      fd_velocity.update();
      filt_velocity.update();
      this.velocity.set(filt_velocity.getDoubleValue());
   }

   public void setVelocity(double qd)
   {
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
      return fd_velocity.getDoubleValue();
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
