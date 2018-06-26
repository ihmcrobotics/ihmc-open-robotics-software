package us.ihmc.valkyrie.fingers.trajectories;

import us.ihmc.yoVariables.providers.DoubleProvider;

public class SinusoidalTrajectory extends GoalPositionTrajectory
{
   public SinusoidalTrajectory(DoubleProvider currentValue)
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
         double diff = finalQ.getValue() - initialQ.getValue();
         double timeStamp = time - delayTime.getValue();
         double timeDiff = trajectoryTime.getValue() - delayTime.getValue();
         q = initialQ.getValue() + 0.5 * diff * (1 - Math.cos(Math.PI * timeStamp / timeDiff));
         qd = 0.5 * diff * Math.PI / timeDiff * Math.sin(Math.PI * timeStamp / timeDiff);
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