package us.ihmc.robotics.math;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class TimestampedVelocityYoVariable extends DoubleYoVariable
{
   private final DoubleYoVariable positionToReferenceOnUpdate;
   private final DoubleYoVariable updatedPosition;
   private final DoubleYoVariable previousPosition;

   private final DoubleYoVariable timeToReferenceOnUpdate;
   private final DoubleYoVariable updatedTimestamp;
   private final DoubleYoVariable previousTimestamp;
   
   private final BooleanYoVariable hasBeenUpdated;
   private final double epsilonChange;

   public TimestampedVelocityYoVariable(String name, String description, DoubleYoVariable positionToReferenceOnUpdate, DoubleYoVariable timeToReferenceOnUpdate, YoVariableRegistry registry,
         double epsilonChange)
   {
      super(name, description, registry);

      this.positionToReferenceOnUpdate = positionToReferenceOnUpdate;
      this.updatedPosition = new DoubleYoVariable(name + "_position", registry);
      this.previousPosition = new DoubleYoVariable(name + "_prevPosition", registry);

      this.timeToReferenceOnUpdate = timeToReferenceOnUpdate;
      this.updatedTimestamp = new DoubleYoVariable(name + "_timestamp", registry);
      this.previousTimestamp = new DoubleYoVariable(name + "_prevTimestamp", registry);
      
      this.hasBeenUpdated = new BooleanYoVariable(name + "_hasBeenUpdated", registry);
      this.epsilonChange = epsilonChange;

      reset();
   }

   public void reset()
   {
      hasBeenUpdated.set(false);
      set(0.0);
   }

   public void update()
   {
	      previousPosition.set(updatedPosition.getDoubleValue());
	      previousTimestamp.set(updatedTimestamp.getDoubleValue());
	      
	      updatedPosition.set(positionToReferenceOnUpdate.getDoubleValue());
	      updatedTimestamp.set(timeToReferenceOnUpdate.getDoubleValue());
	      
      if (hasBeenUpdated.getBooleanValue())
      {
         if (Math.abs(updatedPosition.getDoubleValue() - previousPosition.getDoubleValue()) > epsilonChange)
         {
            double dx = updatedPosition.getDoubleValue() - previousPosition.getDoubleValue();
            double dt = updatedTimestamp.getDoubleValue() - previousTimestamp.getDoubleValue();
            double velocity = dx / dt;
            set(velocity);
         }
      }
      else
      {
         set(0.0);
         hasBeenUpdated.set(true);
      }
   }
   
   public double getPosition()
   {
	   return updatedPosition.getDoubleValue();
   }
   
   public double getTimestamp()
   {
	   return updatedTimestamp.getDoubleValue();
   }
   
   public double getPreviousPosition()
   {
	   return previousPosition.getDoubleValue();
   }
   
   public double getPreviousTimestamp()
   {
	   return previousTimestamp.getDoubleValue();
   }
}
