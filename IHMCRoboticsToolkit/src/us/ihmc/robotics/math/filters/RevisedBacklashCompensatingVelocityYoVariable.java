package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;

/**
 * This does essentially the same as BacklashCompensatingVelocityYoVariable, except does the steps in a different order and 
 * goes sloppy when the output changes sign rather than the finite differences. This works better for a signal with noise on it,
 * which might have a true velocity but due to the noise seems to switch direction often.
 *
 */
public class RevisedBacklashCompensatingVelocityYoVariable extends DoubleYoVariable implements ProcessingYoVariable
{
   private final double dt;

   private final FilteredVelocityYoVariable finiteDifferenceVelocity;
   
   private final DoubleYoVariable alphaVariable;
   private final DoubleYoVariable position;

   private final DoubleYoVariable lastPosition;
   private final BooleanYoVariable hasBeenCalled;

   private final EnumYoVariable<BacklashState> backlashState;
   private final DoubleYoVariable slopTime;

   private final DoubleYoVariable timeSinceSloppy;

   public RevisedBacklashCompensatingVelocityYoVariable(String name, String description, DoubleYoVariable alphaVariable, DoubleYoVariable positionVariable,
           double dt, DoubleYoVariable slopTime, YoVariableRegistry registry)
   {
      super(name, description, registry);
      
      finiteDifferenceVelocity = new FilteredVelocityYoVariable(name + "finiteDifferenceVelocity", "", alphaVariable, dt, registry);
      
      
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      backlashState = new EnumYoVariable<BacklashState>(name + "BacklashState", registry, BacklashState.class, true);
      backlashState.set(null);
      timeSinceSloppy = new DoubleYoVariable(name + "TimeSinceSloppy", registry);

      position = positionVariable;

      this.alphaVariable = alphaVariable;
      this.slopTime = slopTime;

      this.dt = dt;

      lastPosition = new DoubleYoVariable(name + "_lastPosition", registry);

      reset();
   }

   public RevisedBacklashCompensatingVelocityYoVariable(String name, String description, DoubleYoVariable alphaVariable, double dt, DoubleYoVariable slopTime,
           YoVariableRegistry registry)
   {
      super(name, description, registry);
      
      finiteDifferenceVelocity = new FilteredVelocityYoVariable(name + "finiteDifferenceVelocity", "", alphaVariable, dt, registry);

      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      backlashState = new EnumYoVariable<BacklashState>(name + "BacklashState", registry, BacklashState.class, true);
      backlashState.set(null);
      timeSinceSloppy = new DoubleYoVariable(name + "timeInState", registry);

      this.position = null;

      this.alphaVariable = alphaVariable;
      this.slopTime = slopTime;

      this.dt = dt;

      lastPosition = new DoubleYoVariable(name + "_lastPosition", registry);

      reset();
   }

   public void reset()
   {
      hasBeenCalled.set(false);
      backlashState.set(null);
   }

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
         else if (timeSinceSloppy.getDoubleValue() > slopTime.getDoubleValue())
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
         else if (timeSinceSloppy.getDoubleValue() > slopTime.getDoubleValue())
         {
            backlashState.set(BacklashState.FORWARD_OK);
         }

         break;
      }
      } 

      double difference = currentPosition - lastPosition.getDoubleValue();

      double percent = timeSinceSloppy.getDoubleValue() / slopTime.getDoubleValue();
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

      double alpha = alphaVariable.getDoubleValue();
      this.set(alpha * previousFilteredDerivative + (1.0 - alpha) * currentRawDerivative);
   }

   public void setAlpha(double alpha)
   {
      this.alphaVariable.set(alpha);
   }

   public void setSlopTime(double slopTime)
   {
      this.slopTime.set(slopTime);
   }

   private enum BacklashState {BACKWARD_OK, FORWARD_OK, BACKWARD_SLOP, FORWARD_SLOP;}
}
