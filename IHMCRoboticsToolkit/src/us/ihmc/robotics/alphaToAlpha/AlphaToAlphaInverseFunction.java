package us.ihmc.robotics.alphaToAlpha;

public class AlphaToAlphaInverseFunction
{
   private AlphaToAlphaInverseFunction()
   {
   }

   public static double getAlphaPrimeInverse(AlphaToAlphaFunction function, double alphaPrime)
   {
      double leftAlpha = 0.0;
      double rightAlpha = function.getMaxAlpha();
      double leftAlphaPrime = function.getAlphaPrime(leftAlpha);
      double rightAlphaPrime = function.getAlphaPrime(rightAlpha);

      while (true)
      {
         double testAlpha = (leftAlpha + rightAlpha) / 2.0;

         if (Math.abs(rightAlpha - leftAlpha) < 0.001)
            return testAlpha;
         double testAlphaPrime = function.getAlphaPrime(testAlpha);

         if ((leftAlphaPrime <= alphaPrime) && (alphaPrime <= rightAlphaPrime))
         {
            if (testAlphaPrime <= alphaPrime)
               leftAlpha = testAlpha;
            else
               rightAlpha = testAlpha;
         }

         else if ((leftAlphaPrime >= alphaPrime) && (alphaPrime >= rightAlphaPrime))
         {
            if (testAlphaPrime >= alphaPrime)
               leftAlpha = testAlpha;
            else
               rightAlpha = testAlpha;
         }

         else
         {
            System.err.println("Invalid Relation in getAlphaPrimeInverse!!!");

            return testAlpha;
         }
      }
   }
}
