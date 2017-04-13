package us.ihmc.robotics.alphaToAlpha;

/**
 * <p>StretchedSlowAtStartAlphaToAlphaFunction </p>
 *
 * <p>AlphaToAlphaFunction morphing that ensures:
 * a) f(0) = 0, f(1) = 1,
 * b) f'(0) = 0, f'(1) = derivativeAtEnd
 * c) f''(1) = 0 </p>
 *
 * @author IHMC LearningLocomotion Team
 */
public class StretchedSlowAtStartAlphaToAlphaFunction implements AlphaToAlphaFunction
{
   @SuppressWarnings("unused")
   private double derivativeAtEnd;
   private double ratio;
   private double alphaToStartAt;
   private double a2, a3, a4;

   /**
    * Create a StretchedSlowAtStartAlphaToAlphaFunction given nSwing points and nBody points.
    * @param derivativeAtEnd The slope of d-alphaPrime
    */
   public StretchedSlowAtStartAlphaToAlphaFunction(double derivativeAtEnd)
   {
      this.derivativeAtEnd = derivativeAtEnd;
      calculateInternalParameters();


      //    System.out.println("a2 = " + a2 + ", a3 = " + a3 + ", a4 = " + a4 + ", ratio = " + ratio + ", alphaToStartAt = " + alphaToStartAt + ", derivativeAtEnd = " + derivativeAtEnd);
   }

   private void calculateInternalParameters()
   {
      if (derivativeAtEnd < 1.0)
         throw new RuntimeException("We are assuming derivativeAtEnd >= 1.0!!!");

      if (derivativeAtEnd > 2.0)
      {
         alphaToStartAt = (derivativeAtEnd - 2.0) / derivativeAtEnd;
         this.ratio = 2.0;
      }
      else
      {
         alphaToStartAt = 0.0;
         this.ratio = derivativeAtEnd;
      }

      a2 = 6.0 - 3.0 * ratio;
      a3 = -8.0 + 5.0 * ratio;
      a4 = 3.0 - 2.0 * ratio;
   }

   public double getAlphaPrime(double alpha)
   {
      if (alpha < 0.0)
         return 0.0;
      if (alpha > 1.0)
         return 1.0;

      if (alpha < alphaToStartAt)
         return 0.0;
      alpha = (alpha - alphaToStartAt) / (1.0 - alphaToStartAt);

      double ret = a2 * alpha * alpha + a3 * alpha * alpha * alpha + a4 * alpha * alpha * alpha * alpha;

      if (ret < 0.0)
         return 0.0;
      if (ret > 1.0)
         return 1.0;

      return ret;
   }

   public void setDerivativeAtEnd(double derivativeAtEnd)
   {
      this.derivativeAtEnd = derivativeAtEnd;
      calculateInternalParameters();
   }

   public double getMaxAlpha()
   {
      return 1.0;
   }


   public static void main(String[] args)
   {
      new StretchedSlowAtStartAlphaToAlphaFunction(6.0);
   }

   public double getDerivativeAtAlpha(double alpha)
   {
      throw new RuntimeException("Not implemented yet!");
   }

   public double getSecondDerivativeAtAlpha(double alpha)
   {
      throw new RuntimeException("Not implemented yet!");
   }
}
