package us.ihmc.robotics.sensors;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.math.trajectories.CartesianTrajectoryGenerator;

public class CartesianTrajectoryBasedFootSwitch implements TrajectoryBasedFootSwitch
{
   private final String name;
   private final CartesianTrajectoryGenerator trajectoryGenerator;

   private final BooleanYoVariable isSwinging;

   public CartesianTrajectoryBasedFootSwitch(String name, CartesianTrajectoryGenerator trajectoryGenerator, YoVariableRegistry registry)
   {
      this.name = name;
      this.trajectoryGenerator = trajectoryGenerator;

      isSwinging = new BooleanYoVariable(name, registry);
   }

   @Override
   public boolean isSwinging()
   {
      isSwinging.set(!trajectoryGenerator.isDone());

      return isSwinging.getBooleanValue();
   }

   @Override
   public void setIsSwinging(boolean isSwinging)
   {
      this.isSwinging.set(isSwinging);
   }

   @Override
   public boolean switchFoot()
   {
      return !isSwinging();
   }

   @Override
   public void reset()
   {
      isSwinging.set(false);
   }
}
