package us.ihmc.robotics.alphaToAlpha;

import java.util.ArrayList;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class FlatRampFaltAlphaToAlpha implements AlphaToAlphaFunction
{
   private final double initialReturnValue;
   private final double startRampUpValue;
   private final double endRampUpValue;
   private final double finalReturnValue;

   public FlatRampFaltAlphaToAlpha(double initialReturnValue, double startRampUpValue, double endRampUpValue, double finalReturnValue)
   {
      if (endRampUpValue < startRampUpValue)
         throw new RuntimeException("endRampUpValue MUST BE >= startRampUpValue");

      if (startRampUpValue < 0.0)
         throw new RuntimeException("startRampUpValue MUST BE > 0.0");

      if (endRampUpValue < 0.0)
         throw new RuntimeException("endRampUpValue MUST BE > 0.0");

      this.initialReturnValue = initialReturnValue;
      this.startRampUpValue = startRampUpValue;
      this.endRampUpValue = endRampUpValue;
      this.finalReturnValue = finalReturnValue;
   }

   public double getAlphaPrime(double alpha)
   {
      double unsignedAlpha = Math.abs(alpha);

      double scale;
      if (unsignedAlpha <= startRampUpValue)
      {
         scale = initialReturnValue;
      }
      else if (unsignedAlpha < endRampUpValue)
      {
         scale = initialReturnValue + (unsignedAlpha - startRampUpValue) / (endRampUpValue - startRampUpValue) * (finalReturnValue - initialReturnValue);
      }
      else
      {
         scale = finalReturnValue;
      }

      return scale * Math.signum(alpha);
   }

   public double getMaxAlpha()
   {
      throw new RuntimeException("Method not implemented");
   }

   public double getDerivativeAtAlpha(double alpha)
   {
      throw new RuntimeException("Method not implemented");
   }

   public double getSecondDerivativeAtAlpha(double alpha)
   {
      throw new RuntimeException("Method not implemented");
   }

   public static void main(String[] args)
   {
      AlphaToAlphaFunction alphaToAlphaFunction = new FlatRampFaltAlphaToAlpha(1.0, 0.3, 0.9, -0.5);

      ArrayList<Double> xList = new ArrayList<Double>();
      ArrayList<Double> xNewList = new ArrayList<Double>();

      for (double x = -2.0; x <= 2.0; x += 0.01)
      {
         double xNew = alphaToAlphaFunction.getAlphaPrime(x);

         xList.add(x);
         xNewList.add(xNew);
      }

   }
}
