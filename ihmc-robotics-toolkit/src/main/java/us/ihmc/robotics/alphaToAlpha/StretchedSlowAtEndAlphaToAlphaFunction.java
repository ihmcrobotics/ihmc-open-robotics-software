package us.ihmc.robotics.alphaToAlpha;

/**
 * <p>StretchedSlowAtEndAlphaToAlphaFunction </p>
 *
 * <p>AlphaToAlphaFunction morphing that ensures:
 * a) f(0) = 0, f(1) = 1,
 * b) f'(1) = 0, f'(0) = slopeAtStart
 * c) f''(0) = 0 </p>
 *
 * @author IHMC LearningLocomotion Team
 */
public class StretchedSlowAtEndAlphaToAlphaFunction implements AlphaToAlphaFunction
{
   private final StretchedSlowAtStartAlphaToAlphaFunction stretchedSlowAtStartAlphaToAlphaFunction;

   public StretchedSlowAtEndAlphaToAlphaFunction(double slopeAtStart)
   {
      stretchedSlowAtStartAlphaToAlphaFunction = new StretchedSlowAtStartAlphaToAlphaFunction(slopeAtStart);
   }

   public double getAlphaPrime(double alpha)
   {
      double ret = 1.0 - stretchedSlowAtStartAlphaToAlphaFunction.getAlphaPrime(1.0 - alpha);

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

   public void setSlopeAtStart(double slopeAtStart)
   {
      stretchedSlowAtStartAlphaToAlphaFunction.setDerivativeAtEnd(slopeAtStart);
   }

   public double getDerivativeAtAlpha(double alpha)
   {
      throw new RuntimeException("Not implemented yet!");
   }

   public double getSecondDerivativeAtAlpha(double alpha)
   {
      throw new RuntimeException("Not implemented yet!");
   }


//   public static void main(String[] args)
//   {
//      @SuppressWarnings("unused")
//      StretchedSlowAtEndAlphaToAlphaFunction stretchedSlowAtEndAlphaToAlphaFunction = new StretchedSlowAtEndAlphaToAlphaFunction(6.0);
//
//      JFreePlot jFreePlot;
//   }
}
