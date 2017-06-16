package us.ihmc.robotics.sensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.robotics.math.trajectories.Finishable;

public class CartesianTrajectoryBasedFootSwitch implements TrajectoryBasedFootSwitch
{
   private final String name;
   private final Finishable trajectoryGenerator;

   private final YoBoolean isSwinging;

   public CartesianTrajectoryBasedFootSwitch(String name, Finishable trajectoryGenerator, YoVariableRegistry registry)
   {
      this.name = name;
      this.trajectoryGenerator = trajectoryGenerator;

      isSwinging = new YoBoolean(name, registry);
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
