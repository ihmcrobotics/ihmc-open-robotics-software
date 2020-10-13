package us.ihmc.robotics.sensors;

import us.ihmc.robotics.math.trajectories.Finishable;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class CartesianTrajectoryBasedFootSwitch implements TrajectoryBasedFootSwitch
{
   private final String name;
   private final Finishable trajectoryGenerator;

   private final YoBoolean isSwinging;

   public CartesianTrajectoryBasedFootSwitch(String name, Finishable trajectoryGenerator, YoRegistry registry)
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
