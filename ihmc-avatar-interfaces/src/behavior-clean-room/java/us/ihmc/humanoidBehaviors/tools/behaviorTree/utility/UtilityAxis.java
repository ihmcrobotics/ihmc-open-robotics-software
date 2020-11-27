package us.ihmc.humanoidBehaviors.tools.behaviorTree.utility;

import us.ihmc.commons.MathTools;

import java.util.function.DoubleSupplier;

public abstract class UtilityAxis
{
   protected double m;
   protected double k;
   protected double b;
   protected double c;
   private final DoubleSupplier xSupplier;

   public UtilityAxis(double m, double k, double b, double c, DoubleSupplier xSupplier)
   {
      this.m = m;
      this.k = k;
      this.b = b;
      this.c = c;
      this.xSupplier = xSupplier;
   }

   public double calculate()
   {
      return MathTools.clamp(calculateInternal(xSupplier.getAsDouble()), 0.0, 1.0);
   }

   protected abstract double calculateInternal(double x);

   public void setM(double m)
   {
      this.m = m;
   }

   public void setK(double k)
   {
      this.k = k;
   }

   public void setB(double b)
   {
      this.b = b;
   }

   public void setC(double c)
   {
      this.c = c;
   }
}
