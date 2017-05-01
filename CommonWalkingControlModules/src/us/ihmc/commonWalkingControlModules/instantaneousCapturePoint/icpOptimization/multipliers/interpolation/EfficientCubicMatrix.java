package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.MathTools;

public class EfficientCubicMatrix extends DenseMatrix64F
{
   private double duration;
   private double duration2;
   private double duration3;

   public EfficientCubicMatrix()
   {
      super(1, 4);
   }

   public void setSegmentDuration(double duration)
   {
      this.duration = duration;
      this.duration2 = Math.pow(duration, 2.0);
      this.duration3 = Math.pow(duration, 3.0);
   }

   public void update(double timeInCurrentState)
   {
      timeInCurrentState = MathTools.clamp(timeInCurrentState, 0.0, duration);
      double timeInCurrentState2 = Math.pow(timeInCurrentState, 2.0);
      double timeInCurrentState3 = Math.pow(timeInCurrentState, 3.0);

      set(0, 0, 2.0 * timeInCurrentState3 / duration3 - 3.0 * timeInCurrentState2 / duration2 + 1.0);
      set(0, 1, timeInCurrentState3 / duration2 - 2.0 * timeInCurrentState2 / duration + timeInCurrentState);
      set(0, 2, -(get(0, 0) - 1.0));
      set(0, 3, timeInCurrentState3 / duration2 - timeInCurrentState2 / duration);
   }
}
