package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;


public class BacklashCompensatingVelocityYoVariable extends DoubleYoVariable implements ProcessingYoVariable
{
   private final double dt;

   private final DoubleYoVariable alphaVariable;
   private final DoubleYoVariable position;

   private final DoubleYoVariable lastPosition;
   private final BooleanYoVariable hasBeenCalled;

   private final EnumYoVariable<BacklashState> backlashState;
   private final DoubleYoVariable slopTime;

   private final DoubleYoVariable timeInState;

   public BacklashCompensatingVelocityYoVariable(String name, String description, DoubleYoVariable alphaVariable, DoubleYoVariable positionVariable, double dt,
         DoubleYoVariable slopTime, YoVariableRegistry registry)
   {
      super(name, description, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      backlashState = new EnumYoVariable<BacklashState>(name + "BacklashState", registry, BacklashState.class, true);
      backlashState.set(null);
      timeInState = new DoubleYoVariable(name + "TimeInState", registry);

      position = positionVariable;

      this.alphaVariable = alphaVariable;
      this.slopTime = slopTime;

      this.dt = dt;

      lastPosition = new DoubleYoVariable(name + "_lastPosition", registry);

      reset();
   }

   public BacklashCompensatingVelocityYoVariable(String name, String description, DoubleYoVariable alphaVariable, double dt, DoubleYoVariable slopTime,
         YoVariableRegistry registry)
   {
      super(name, description, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", registry);

      backlashState = new EnumYoVariable<BacklashState>(name + "BacklashState", registry, BacklashState.class, true);
      backlashState.set(null);
      timeInState = new DoubleYoVariable(name + "timeInState", registry);

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

   //   public void updateForAngles()
   //   {
   //      if (position == null)
   //      {
   //         throw new NullPointerException("YoFilteredVelocityVariable must be constructed with a non null "
   //                                        + "position variable to call update(), otherwise use update(double)");
   //      }
   //
   //      updateForAngles(position.getDoubleValue());
   //   }

   public void update(double currentPosition)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         lastPosition.set(currentPosition);
         set(0.0);
      }

      timeInState.add(dt);

      boolean sloppy = false;

      double difference = currentPosition - lastPosition.getDoubleValue();
      
      if (backlashState.getEnumValue() == null)
      {
         if (difference > 0.0)
            backlashState.set(BacklashState.FORWARD_OK);
         else if (difference < 0.0)
            backlashState.set(BacklashState.BACKWARD_OK);
      }
      else
      {
         switch (backlashState.getEnumValue())
         {
            case BACKWARD_OK:
            {
               if (difference > 0.0)
               {
                  backlashState.set(BacklashState.FORWARD_SLOP);
                  sloppy = true;
                  timeInState.set(0.0);
               }
               break;
            }
            case FORWARD_OK:
            {
               if (difference < 0.0)
               {
                  backlashState.set(BacklashState.BACKWARD_SLOP);
                  sloppy = true;
                  timeInState.set(0.0);
               }

               break;
            }
            case BACKWARD_SLOP:
            {
               sloppy = true;

               if (difference > 0.0)
               {
                  backlashState.set(BacklashState.FORWARD_SLOP);
                  sloppy = true;
                  timeInState.set(0.0);
               }
               else if (timeInState.getDoubleValue() > slopTime.getDoubleValue())
               {
                  backlashState.set(BacklashState.BACKWARD_OK);
                  sloppy = false;
                  timeInState.set(0.0);
               }

               break;
            }
            case FORWARD_SLOP:
            {
               sloppy = true;

               if (difference < 0.0)
               {
                  backlashState.set(BacklashState.BACKWARD_SLOP);
                  sloppy = true;
                  timeInState.set(0.0);
               }
               else if (timeInState.getDoubleValue() > slopTime.getDoubleValue())
               {
                  backlashState.set(BacklashState.FORWARD_OK);
                  sloppy = false;
                  timeInState.set(0.0);
               }
               break;
            }
         }
      }

      if (sloppy)
      {
         double percent = timeInState.getDoubleValue() / slopTime.getDoubleValue();
         percent = MathTools.clamp(percent, 0.0, 1.0);
         if (Double.isNaN(percent))
            percent = 1.0;

         double scaleFactor = percent;

         difference = scaleFactor * difference;
      }

      updateUsingDifference(difference);

      lastPosition.set(currentPosition);

   }

   //   public void updateForAngles(double currentPosition)
   //   {
   //      if (!hasBeenCalled.getBooleanValue())
   //      {
   //         hasBeenCalled.set(true);
   //         lastPosition.set(currentPosition);
   //         set(0.0);
   //      }
   //
   //      double difference = AngleTools.computeAngleDifferenceMinusPiToPi(currentPosition, lastPosition.getDoubleValue());
   //
   //      updateUsingDifference(difference);
   //
   //      lastPosition.set(currentPosition);
   //   }

   private void updateUsingDifference(double difference)
   {
      double previousFilteredDerivative = getDoubleValue();
      double currentRawDerivative = difference / dt;

      double alpha = alphaVariable.getDoubleValue();
      set(alpha * previousFilteredDerivative + (1.0 - alpha) * currentRawDerivative);
   }

   public void setAlpha(double alpha)
   {
      this.alphaVariable.set(alpha);
   }

   public void setSlopTime(double slopTime)
   {
      this.slopTime.set(slopTime);
   }

   private enum BacklashState
   {
      BACKWARD_OK, FORWARD_OK, BACKWARD_SLOP, FORWARD_SLOP;
   }
}
