package us.ihmc.robotics.math.filters;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

/**
 * This does essentially the same as BacklashCompensatingVelocityYoVariable, except does the steps in a different order and 
 * goes sloppy when the output changes sign rather than the finite differences. This works better for a signal with noise on it,
 * which might have a true velocity but due to the noise seems to switch direction often.
 *
 */
public class RevisedBacklashCompensatingVelocityYoVariable extends YoDouble implements ProcessingYoVariable
{
   private final double dt;

   private final FilteredVelocityYoVariable finiteDifferenceVelocity;

   private final DoubleProvider alphaVariable;
   private final YoDouble position;

   private final YoDouble lastPosition;
   private final YoBoolean hasBeenCalled;

   private final YoEnum<BacklashState> backlashState;
   private final DoubleProvider slopTime;

   private final YoDouble timeSinceSloppy;

   public RevisedBacklashCompensatingVelocityYoVariable(String name, String description, DoubleProvider alphaVariable, YoDouble positionVariable,
           double dt, DoubleProvider slopTime, YoVariableRegistry registry)
   {
      super(name, description, registry);
      
      finiteDifferenceVelocity = new FilteredVelocityYoVariable(name + "finiteDifferenceVelocity", "", alphaVariable, dt, registry);
      
      
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);

      backlashState = new YoEnum<BacklashState>(name + "BacklashState", registry, BacklashState.class, true);
      backlashState.set(null);
      timeSinceSloppy = new YoDouble(name + "TimeSinceSloppy", registry);

      position = positionVariable;

      this.alphaVariable = alphaVariable;
      this.slopTime = slopTime;

      this.dt = dt;

      lastPosition = new YoDouble(name + "_lastPosition", registry);

      reset();
   }

   public RevisedBacklashCompensatingVelocityYoVariable(String name, String description, YoDouble alphaVariable, double dt, YoDouble slopTime,
           YoVariableRegistry registry)
   {
      super(name, description, registry);
      
      finiteDifferenceVelocity = new FilteredVelocityYoVariable(name + "finiteDifferenceVelocity", "", alphaVariable, dt, registry);

      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);

      backlashState = new YoEnum<BacklashState>(name + "BacklashState", registry, BacklashState.class, true);
      backlashState.set(null);
      timeSinceSloppy = new YoDouble(name + "timeInState", registry);

      this.position = null;

      this.alphaVariable = alphaVariable;
      this.slopTime = slopTime;

      this.dt = dt;

      lastPosition = new YoDouble(name + "_lastPosition", registry);

      reset();
   }

   @Override
   public void reset()
   {
      hasBeenCalled.set(false);
      backlashState.set(null);
   }

   @Override
   public void update()
   {
      if (position == null)
      {
         throw new NullPointerException("YoFilteredVelocityVariable must be constructed with a non null "
                                        + "position variable to call update(), otherwise use update(double)");
      }

      update(position.getDoubleValue());
   }

   public void update(double currentPosition)
   {
      if (backlashState.getEnumValue() == null)
      {
         backlashState.set(BacklashState.FORWARD_OK);
      }
      
      finiteDifferenceVelocity.update(currentPosition);
      
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         lastPosition.set(currentPosition);
         set(0.0);
      }

      timeSinceSloppy.add(dt);

      double velocityFromFiniteDifferences = finiteDifferenceVelocity.getDoubleValue();

      switch (backlashState.getEnumValue())
      {
      case BACKWARD_OK :
      {
         if (velocityFromFiniteDifferences > 0.0)
         {
            timeSinceSloppy.set(0.0);
//            this.set(0.0);
            backlashState.set(BacklashState.FORWARD_SLOP);
         }

         break;
      }

      case FORWARD_OK :
      {
         if (velocityFromFiniteDifferences < 0.0)
         {
            timeSinceSloppy.set(0.0);
//            this.set(0.0);
            backlashState.set(BacklashState.BACKWARD_SLOP);
         }

         break;
      }

      case BACKWARD_SLOP :
      {
         if (velocityFromFiniteDifferences > 0.0)
         {
            timeSinceSloppy.set(0.0);
//            this.set(0.0);
            backlashState.set(BacklashState.FORWARD_SLOP);
         }
         else if (timeSinceSloppy.getDoubleValue() > slopTime.getValue())
         {
            backlashState.set(BacklashState.BACKWARD_OK);
         }

         break;
      }

      case FORWARD_SLOP :
      {
         if (velocityFromFiniteDifferences < 0.0)
         {
            timeSinceSloppy.set(0.0);
//            this.set(0.0);
            backlashState.set(BacklashState.BACKWARD_SLOP);
         }
         else if (timeSinceSloppy.getDoubleValue() > slopTime.getValue())
         {
            backlashState.set(BacklashState.FORWARD_OK);
         }

         break;
      }
      } 

      double difference = currentPosition - lastPosition.getDoubleValue();

      double percent = timeSinceSloppy.getDoubleValue() / slopTime.getValue();
      percent = MathTools.clamp(percent, 0.0, 1.0);
      if (Double.isNaN(percent))
         percent = 1.0;

      double scaleFactor = percent;
      difference = scaleFactor * difference;
      
      this.updateUsingDifference(difference);
      lastPosition.set(currentPosition);
   }
   
   private void updateUsingDifference(double difference)
   {
      double previousFilteredDerivative = getDoubleValue();
      double currentRawDerivative = difference / dt;

      double alpha = alphaVariable.getValue();
      this.set(alpha * previousFilteredDerivative + (1.0 - alpha) * currentRawDerivative);
   }

   private enum BacklashState {BACKWARD_OK, FORWARD_OK, BACKWARD_SLOP, FORWARD_SLOP;}
}
