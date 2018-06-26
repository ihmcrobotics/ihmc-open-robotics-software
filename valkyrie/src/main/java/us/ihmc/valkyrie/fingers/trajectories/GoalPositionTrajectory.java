package us.ihmc.valkyrie.fingers.trajectories;

import us.ihmc.robotics.math.trajectories.DoubleTrajectoryGenerator;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;

public abstract class GoalPositionTrajectory implements DoubleTrajectoryGenerator
{
   private final SettableDoubleProvider max = new SettableDoubleProvider(Double.MAX_VALUE);
   private final SettableDoubleProvider min = new SettableDoubleProvider(Double.MIN_VALUE);

   private final SettableDoubleProvider currentTime;

   protected final SettableDoubleProvider currentQ;
   protected final SettableDoubleProvider currentQd;

   protected final SettableDoubleProvider initialQ;
   protected final SettableDoubleProvider finalQ;

   protected final SettableDoubleProvider trajectoryTime;
   protected final SettableDoubleProvider delayTime;

   public GoalPositionTrajectory(double currentValue)
   {
      currentTime = new SettableDoubleProvider();

      currentQ = new SettableDoubleProvider(currentValue);
      currentQd = new SettableDoubleProvider();
      initialQ = new SettableDoubleProvider(currentValue);
      finalQ = new SettableDoubleProvider();

      trajectoryTime = new SettableDoubleProvider();
      delayTime = new SettableDoubleProvider();
   }

   public void setUpperLimit(double upperLimit)
   {
      max.setValue(upperLimit);
   }

   public void setLowerLimit(double lowerLimit)
   {
      min.setValue(lowerLimit);
   }

   public void setGoalPosition(double trajectoryTime, double delayTime, double goalPosition)
   {
      this.trajectoryTime.setValue(trajectoryTime);
      this.delayTime.setValue(delayTime);
      this.finalQ.setValue(goalPosition);

      this.currentTime.setValue(0.0);
   }

   protected abstract void update(double time);

   private void clamp()
   {
      if (currentQ.getValue() > max.getValue())
      {
         currentQ.setValue(max.getValue());
         currentQd.setValue(0.0);
      }
      if (currentQ.getValue() < min.getValue())
      {
         currentQ.setValue(min.getValue());
         currentQd.setValue(0.0);
      }
   }

   @Override
   public void initialize()
   {
      initialQ.setValue(currentQ.getValue());
   }

   @Override
   public void compute(double time)
   {
      currentTime.setValue(time);
      update(time);
      clamp();
   }

   @Override
   public boolean isDone()
   {
      return trajectoryTime.getValue() <= currentTime.getValue();
   }

   @Override
   public double getValue()
   {
      return currentQ.getValue();
   }

   @Override
   public double getVelocity()
   {
      return currentQd.getValue();
   }

   @Override
   public double getAcceleration()
   {
      return 0;
   }
}
