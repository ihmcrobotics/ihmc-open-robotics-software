package us.ihmc.robotics.alphaToAlpha;

/**
 * <p>StretchedSlowInMiddleAlphaToAlphaFunction </p>
 *
 * <p>AlphaToAlphaFunction morphing that ensures:
 * a) f(0) = 0, f(1) = 1,
 * b) f'(0) = derivativeAtStartAndEnd, f'(1) = derivativeAtStartAndEnd
 * c) f''(0) = 0, f''(1) = 0 </p>
 *
 * @author IHMC LearningLocomotion Team
 */
public class StretchedSlowInMiddleAlphaToAlphaFunction implements AlphaToAlphaFunction
{
   @SuppressWarnings("unused")
   private final double derivativeAtStartAndEnd;
   private final double ratio;
   private final double alphaHoldSteady;
   private final double a1, a3, a4, a5;

   private static final double ratioRequiringStop = 15.0 / 7.0;

   /**
    * Create a StretchedSlowInMiddleAlphaToAlphaFunction given nSwing points and nBody points.
    * @param derivativeAtStartAndEnd The slope of alphaPrime
    */
   public StretchedSlowInMiddleAlphaToAlphaFunction(double derivativeAtStartAndEnd)
   {
      if (Double.isNaN(derivativeAtStartAndEnd))
         throw new RuntimeException("Double.isNaN(derivativeAtStartAndEnd)");
      if (Double.isInfinite(derivativeAtStartAndEnd))
         throw new RuntimeException("Double.isInfinite(derivativeAtStartAndEnd)");


      this.derivativeAtStartAndEnd = derivativeAtStartAndEnd;

      if (derivativeAtStartAndEnd < 1.0)
         throw new RuntimeException("We are assuming derivativeAtStartAndEnd >= 1.0!!!");

      if (derivativeAtStartAndEnd > ratioRequiringStop)
      {
         alphaHoldSteady = 0.5 * ratioRequiringStop / derivativeAtStartAndEnd;
         this.ratio = ratioRequiringStop;
      }
      else
      {
         alphaHoldSteady = 0.5;
         this.ratio = derivativeAtStartAndEnd;
      }

      a1 = ratio;
      a3 = -10.0 * ratio + 10.0;
      a4 = 15.0 * ratio - 15.0;
      a5 = -6.0 * ratio + 6.0;

//    // Checks:
//    double y_1 = a1 + a3 + a4 + a5;
//    double zero_check = 6*a3+12*a4+20*a5;
//
//    System.err.println("y_1 = " + y_1);
//    System.err.println("zero_check = " + zero_check);


//    System.out.println("a2 = " + a2 + ", a3 = " + a3 + ", a4 = " + a4 + ", ratio = " + ratio + ", alphaToStartAt = " + alphaToStartAt + ", derivativeAtEnd = " + derivativeAtEnd);
   }

   public double getAlphaPrime(double alpha)
   {
      if (alpha < 0.0)
         return 0.0;
      if (alpha > 1.0)
         return 1.0;

//    if (alpha < alphaToStartAt) return 0.0;

      if (alpha <= alphaHoldSteady)
      {
         alpha = 0.5 * alpha / alphaHoldSteady;
      }
      else if (alpha > 1.0 - alphaHoldSteady)
      {
         alpha = 0.5 + 0.5 * ((alpha - (1.0 - alphaHoldSteady)) / alphaHoldSteady);
      }
      else
      {
         alpha = 0.5;
      }


      double ret = a1 * alpha + a3 * alpha * alpha * alpha + a4 * alpha * alpha * alpha * alpha + a5 * alpha * alpha * alpha * alpha * alpha;

      if (ret < 0.0)
         return 0.0;
      if (ret > 1.0)
         return 1.0;

      return ret;
   }

   public double getMaxAlpha()
   {
      return 1.0;
   }

   public double getDerivativeAtAlpha(double alpha)
   {
      if ((alpha < 0.0) || (alpha > 1.0))
         return 0.0;

      double scaling;
      if (alpha <= alphaHoldSteady)
      {
         alpha = 0.5 * alpha / alphaHoldSteady;
         scaling = 0.5 / alphaHoldSteady;
      }
      else if (alpha > 1.0 - alphaHoldSteady)
      {
         alpha = 0.5 + 0.5 * ((alpha - (1.0 - alphaHoldSteady)) / alphaHoldSteady);
         scaling = 0.5 / alphaHoldSteady;
      }
      else
      {
         return 0.0;
      }

      return scaling
             * (a1 /* + 2.0 * a2 * alpha = 0 */ + 3.0 * a3 * alpha * alpha + 4.0 * a4 * alpha * alpha * alpha + 5.0 * a5 * alpha * alpha * alpha * alpha);
   }

   public double getSecondDerivativeAtAlpha(double alpha)
   {
      if ((alpha < 0.0) || (alpha > 1.0))
         return 0.0;

      double scaling;
      if (alpha <= alphaHoldSteady)
      {
         alpha = 0.5 * alpha / alphaHoldSteady;
         scaling = 0.5 / alphaHoldSteady;
      }
      else if (alpha > 1.0 - alphaHoldSteady)
      {
         alpha = 0.5 + 0.5 * ((alpha - (1.0 - alphaHoldSteady)) / alphaHoldSteady);
         scaling = 0.5 / alphaHoldSteady;
      }
      else
      {
         return 0.0;
      }

      return scaling * scaling * ( /* 2.0 * a2 = 0 */+6.0 * a3 * alpha + 12.0 * a4 * alpha * alpha + 20.0 * a5 * alpha * alpha * alpha);
   }

   public static void main(String[] args)
   {
      new StretchedSlowInMiddleAlphaToAlphaFunction(3.0);    // 2.14286);
   }
}
