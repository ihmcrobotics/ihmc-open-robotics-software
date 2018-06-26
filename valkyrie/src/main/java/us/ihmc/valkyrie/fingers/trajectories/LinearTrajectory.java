package us.ihmc.valkyrie.fingers.trajectories;

import us.ihmc.yoVariables.providers.DoubleProvider;

public class LinearTrajectory extends GoalPositionTrajectory
{
   public LinearTrajectory(DoubleProvider currentValue)
   {
      super(currentValue);
   }

   @Override
   protected void update(double time)
   {
      double q, qd = 0.0;

      if (time <= delayTime.getValue())
      {
         q = initialQ.getValue();
         qd = 0.0;
      }
      else if (delayTime.getValue() < time && time < trajectoryTime.getValue())
      {
         q = initialQ.getValue()
               + (finalQ.getValue() - initialQ.getValue()) * (time - delayTime.getValue()) / (trajectoryTime.getValue() - delayTime.getValue());
         qd = 1.0 / (finalQ.getValue() - initialQ.getValue()) * (trajectoryTime.getValue() - delayTime.getValue());
      }
      else
      {
         q = finalQ.getValue();
         qd = 0.0;
      }

      currentQ.setValue(q);
      currentQd.setValue(qd);
   }
}