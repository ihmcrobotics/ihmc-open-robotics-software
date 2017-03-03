package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class ConstantForceTrajectoryGenerator implements DoubleTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable force;
   private final DoubleYoVariable finalTime;
   private final DoubleYoVariable time;

   public ConstantForceTrajectoryGenerator(String namePrefix, double force, double finalTime, YoVariableRegistry parentRegistry)
   {
      MathTools.checkIntervalContains(finalTime, 0.0, Double.POSITIVE_INFINITY);

      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.force = new DoubleYoVariable("force", registry);
      this.finalTime = new DoubleYoVariable("finalTime", registry);
      this.time = new DoubleYoVariable("time", registry);
      this.force.set(force);
      this.finalTime.set(finalTime);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      time.set(0.0);
   }

   public void compute(double time)
   {
      this.time.set(time);
   }

   public boolean isDone()
   {
      return time.getDoubleValue() > finalTime.getDoubleValue();
   }

   public double getValue()
   {
      return force.getDoubleValue();
   }

   public double getVelocity()
   {
      return 0.0;
   }

   public double getAcceleration()
   {
      return 0.0;
   }
}
