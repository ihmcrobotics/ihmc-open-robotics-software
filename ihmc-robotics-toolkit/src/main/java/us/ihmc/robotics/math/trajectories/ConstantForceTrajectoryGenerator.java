package us.ihmc.robotics.math.trajectories;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ConstantForceTrajectoryGenerator implements DoubleTrajectoryGenerator
{
   private final YoRegistry registry;
   private final YoDouble force;
   private final YoDouble finalTime;
   private final YoDouble time;

   public ConstantForceTrajectoryGenerator(String namePrefix, double force, double finalTime, YoRegistry parentRegistry)
   {
      MathTools.checkIntervalContains(finalTime, 0.0, Double.POSITIVE_INFINITY);

      this.registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      this.force = new YoDouble("force", registry);
      this.finalTime = new YoDouble("finalTime", registry);
      this.time = new YoDouble("time", registry);
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
