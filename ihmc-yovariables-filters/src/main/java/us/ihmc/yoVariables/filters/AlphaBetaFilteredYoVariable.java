package us.ihmc.yoVariables.filters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * <p>
 * An {@code AlphaBetaFilteredYoVariable} is a filtered version of {@link YoDouble}, and estimates the velocity of some value it is tracking.
 * Either a {@link YoDouble} holding the position value is passed in to the
 * constructor and {@link #update()} is called every tick, or {@link #update(double)} is
 * called every tick. The {@code AlphaBetaFilteredYoVariable} updates its value
 * with the current filtered velocity using
 * <p>
 * xp  =  x + (dt) v            // angular position prediction
 * x+  =  xp + alpha (xmeas - xp)   // adjusted angular position estimate
 * v+  =  v + beta (xmeas - xp)    // adjusted velocity estimate
 *
 * </p>
 * <pre>
 *
 *         For complete reference see:
 *                http://www.mstarlabs.com/control/engspeed.html#Ref2
 *
 *         </pre>
 */
public class AlphaBetaFilteredYoVariable extends YoDouble
{
   private double alpha = 0.0, beta = 0.0;

   private final double DT;
   private YoDouble alphaVariable = null;
   private YoDouble betaVariable = null;

   private final YoDouble positionState;
   private final YoDouble xMeasuredVariable;

   public AlphaBetaFilteredYoVariable(String name,
                                      YoRegistry registry,
                                      double alpha,
                                      double beta,
                                      YoDouble positionVariable,
                                      YoDouble xMeasuredVariable,
                                      double DT)
   {
      super(name, registry);
      this.alpha = alpha;
      this.beta = beta;
      this.DT = DT;
      this.positionState = positionVariable;
      this.xMeasuredVariable = xMeasuredVariable;

      reset();
   }

   public void reset()
   {
   }

   public YoDouble getPositionEstimation()
   {
      return positionState;
   }

   public YoDouble getVelocityEstimation()
   {
      return this;
   }

   public void update()
   {
      update(positionState.getDoubleValue());
   }

   public void update(double position)
   {
      if (alphaVariable != null)
      {
         alpha = alphaVariable.getDoubleValue();
      }

      if (betaVariable != null)
      {
         beta = betaVariable.getDoubleValue();
      }

      double velocity = this.getDoubleValue();

      double prediction = position + DT * velocity; // xp  =  x + (dt) v,          position prediction
      double error = xMeasuredVariable.getDoubleValue() - prediction;
      double newPosition = prediction + alpha * error; // x+  =  xp + alpha (xmeas - xp), adjusted position estimate
      double newVelocity = velocity + beta * error; // v+  =  v + beta (xmeas - xp),  adjusted velocity estimate

      positionState.set(newPosition);
      this.set(newVelocity); // filter updates its YoVariable value to velocity by convention.
   }
}
