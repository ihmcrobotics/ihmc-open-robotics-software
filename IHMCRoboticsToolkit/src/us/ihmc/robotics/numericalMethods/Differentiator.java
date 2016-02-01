package us.ihmc.robotics.numericalMethods;

public class Differentiator
{
   private boolean hasBeenUpdated = false;
   private double previous = Double.NaN;
   private double output = Double.NaN;
   private double dt;

   public Differentiator(double dt)
   {
      this.dt = dt;
   }

   public void update(double input)
   {
      if (hasBeenUpdated)
      {
         output = (input - previous) / dt;
         previous = input;
      }
      else
      {
         reset(input);
      }

      hasBeenUpdated = true;
   }

   public double val()
   {
      return output;
   }

   public void reset(double resetVal)
   {
      previous = resetVal;
      output = 0.0;
   }
}
