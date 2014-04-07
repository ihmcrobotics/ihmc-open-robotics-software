package us.ihmc.valkyrie.kinematics;

import us.ihmc.valkyrie.ValkyrieControllerFactory;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.filter.BacklashCompensatingVelocityYoVariable;
import com.yobotics.simulationconstructionset.util.math.filter.FilteredVelocityYoVariable;

public class ValkyrieJoint
{
   private static final double DT = ValkyrieControllerFactory.getEstimatorDT();
   
   private final String name;
   private final DoubleYoVariable position;
   private final DoubleYoVariable velocity;
   private final FilteredVelocityYoVariable fd_velocity;
   private final AlphaFilteredYoVariable filt_velocity;
   private final BacklashCompensatingVelocityYoVariable bl_velocity;
   private final AlphaFilteredYoVariable filt_bl_velocity;
   private final DoubleYoVariable effort;

   private final DoubleYoVariable desiredPosition;
   private final DoubleYoVariable desiredVelocity;
   private final DoubleYoVariable desiredEffort;
   
   private final BooleanYoVariable useTwoPoleFiltering;
   
   private final BooleanYoVariable useBacklashComp;
   private final DoubleYoVariable slopTimeForBacklashComp;

   public ValkyrieJoint(String name, YoVariableRegistry registry)
   {
      this(name, null, null, null, null, registry);
   }
   
   public ValkyrieJoint(String name, DoubleYoVariable alpha, BooleanYoVariable useTwoPoleFiltering, BooleanYoVariable useBacklashComp, DoubleYoVariable slopTimeForBacklashComp, YoVariableRegistry registry)
   {
      this.name = name;
      position = new DoubleYoVariable(name + "_q", registry);
      velocity = new DoubleYoVariable(name + "_qd", registry);
      
      this.useTwoPoleFiltering = useTwoPoleFiltering;
      this.useBacklashComp = useBacklashComp;
      this.slopTimeForBacklashComp = slopTimeForBacklashComp;
      
      if (alpha != null)
      {
         fd_velocity = new FilteredVelocityYoVariable(name + "_fd_qd", "", alpha, position, DT, registry);
         filt_velocity = new AlphaFilteredYoVariable(name + "_filt_qd", registry, alpha, fd_velocity);
      }
      else
      {
         fd_velocity = new FilteredVelocityYoVariable(name + "_fd_qd", "", 0.0, position, DT, registry);
         filt_velocity = new AlphaFilteredYoVariable(name + "_filt_qd", registry, 0.0, fd_velocity);
      }
      
      if (useBacklashComp != null)
      {
         if (slopTimeForBacklashComp == null)
            throw new RuntimeException("Ooops");
         
         bl_velocity = new BacklashCompensatingVelocityYoVariable(name + "_bl_qd", "", alpha, position, DT, slopTimeForBacklashComp, registry);
         filt_bl_velocity = new AlphaFilteredYoVariable(name + "_filt_bl_qd", registry, alpha, fd_velocity);
      }
      else
      {
         bl_velocity = null;
         filt_bl_velocity = null;
      }
      
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
      bl_velocity.update();
      filt_bl_velocity.update();
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
      boolean useTwoPoleFiltering = this.useTwoPoleFiltering != null && this.useTwoPoleFiltering.getBooleanValue();
      boolean useBacklashComp = this.useBacklashComp != null && this.useBacklashComp.getBooleanValue();
      
      if (useBacklashComp)
      {
         if (useTwoPoleFiltering)
            return filt_bl_velocity.getDoubleValue();
         else
            return bl_velocity.getDoubleValue();
      }
      else
      {
         if (useTwoPoleFiltering)
            return filt_velocity.getDoubleValue();
         else
            return fd_velocity.getDoubleValue();
      }
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
