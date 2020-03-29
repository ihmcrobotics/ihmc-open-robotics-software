package us.ihmc.robotics.math.trajectories;

import gnu.trove.list.array.TDoubleArrayList;
import org.apache.commons.math3.util.CombinatoricsUtils;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BezierCurve
{
   private final TDoubleArrayList knots = new TDoubleArrayList();
   private double startT = Double.POSITIVE_INFINITY;
   private double endT = Double.NEGATIVE_INFINITY;

   private double value;

   public BezierCurve()
   {
   }

   public void reset()
   {
      startT = Double.POSITIVE_INFINITY;
      endT = Double.NEGATIVE_INFINITY;
      knots.clear();
   }

   public void addPoint(double t, double value)
   {
      startT = Math.min(t, startT);
      endT = Math.max(t, endT);
      knots.add(value);
   }

   public void compute(double t)
   {
      double alpha = (t - startT) / (endT - startT);
      value = 0.0;
      int n = knots.size() - 1;
      for (int i = 0; i < knots.size(); i++)
      {
         double coefficient = computeBinomialCoefficient(n, i);
         coefficient *= MathTools.pow(1.0 - alpha, n - i) * MathTools.pow(alpha, i);
         value += coefficient * knots.get(i);
      }
   }

   public double getPosition()
   {
      return value;
   }

   private static double computeBinomialCoefficient(int n, int i)
   {
      return CombinatoricsUtils.factorial(n) / (CombinatoricsUtils.factorial(i) * CombinatoricsUtils.factorial(n - i));
   }
}
