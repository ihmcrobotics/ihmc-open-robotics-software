package us.ihmc.simulationconstructionset.util.math.alphaToAlpha;

import java.util.ArrayList;


/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class SinusoidalAlphaToAlpha
{
   private static final double EPSILON = 1e-10;
   private double minValue;
   private double maxValue;
   private boolean invert;

   public SinusoidalAlphaToAlpha(double minValue, double maxValue, boolean invert)
   {
      this.minValue = minValue;
      this.maxValue = maxValue;
      this.invert = invert;

      if (maxValue <= minValue)
      {
         throw new RuntimeException("maxValue must be greater than the minValue.");
      }
   }

   public double getAlpha(double value)
   {
      double ret;

      if (value < (minValue - EPSILON))
         ret = 0.0;

      double inAlpha = (value - minValue) / (maxValue - minValue);

      if (inAlpha <= 0.0)
      {
         ret = 0.0;
      }
      else if (inAlpha <= 1.0)
      {
         ret = 0.5 * (1.0 - Math.cos(inAlpha * Math.PI));
      }
      else
      {
         ret = 1.0;
      }

      if (!invert)
      {
         return ret;
      }
      else
      {
         return 1.0 - ret;
      }
   }

   public static void main(String[] args)
   {
      ArrayList<Double> in = new ArrayList<Double>();
      ArrayList<Double> out = new ArrayList<Double>();

      double maxValue = 11.5;
      double minValue = -1.1;
      SinusoidalAlphaToAlpha testSinusoidalAlphaToAlpha = new SinusoidalAlphaToAlpha(minValue, maxValue, false);

      for (double inValue = minValue - 1.0; inValue <= (maxValue + 1.0); inValue = inValue + 0.0243)
      {
         in.add(inValue);
         out.add(testSinusoidalAlphaToAlpha.getAlpha(inValue));
      }

//      PlotGraph2d pg = PlotGraph2d.createPlotGraph2d(in, out);
//      pg.plot();


      // check
      if (testSinusoidalAlphaToAlpha.getAlpha(minValue) == 0.0)
      {
         System.out.println("got Zero");
      }
      else
      {
         System.out.println("Should have gotten zero.");
      }

   }




}
