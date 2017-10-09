package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */

/**
 * @author thutcheson
 *         </p>
 *         <p>
 *         </p>
 *         <p>
 *         An AlphaBetaFilteredYoVariable is a filtered version of an input YoVar.
 *         Either a YoVariable holding the unfiltered val is passed in to the
 *         constructor and update() is called every tick, or update(double) is
 *         called every tick. The AlphaBetaFilteredYoVariable updates it's val
 *         with the current filtered velocity using
 *
 *         xp  =  x + (dt) v            // angular position prediction
 *         x+  =  xp + alpha (xmeas - xp)   // adjusted angular position estimate
 *         v+  =  v + beta (xmeas - xp)    // adjusted velocity estimate
 *
 *         </p>
 *         <pre>
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

   public AlphaBetaFilteredYoVariable(String name, YoVariableRegistry registry, double alpha, double beta, YoDouble positionVariable,
         YoDouble xMeasuredVariable, double DT)
   {
      super(name, registry);
      this.alpha = alpha;
      this.beta = beta;
      this.DT = DT;
      this.positionState = positionVariable;
      this.xMeasuredVariable = xMeasuredVariable;

      reset();
   }

   public AlphaBetaFilteredYoVariable(String name, YoVariableRegistry registry, YoDouble alphaVariable, YoDouble betaVariable,
         YoDouble positionVariable, YoDouble xMeasuredVariable, double DT)
   {
      super(name, registry);
      this.alphaVariable = alphaVariable;
      this.betaVariable = betaVariable;
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
