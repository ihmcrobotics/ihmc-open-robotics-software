package us.ihmc.robotics.math;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class YoSolvePseudoInverseSVDWithDampedLeastSquaresNearSingularities implements LinearSolver<DenseMatrix64F>
{
   private final SingularValueDecomposition<DenseMatrix64F> svd;
   private final DenseMatrix64F pseudoInverse = new DenseMatrix64F(1, 1);

   private final YoVariableRegistry registry;
   private final DoubleYoVariable mu;
   private final DoubleYoVariable firstSingularValueThreshold;
   private final DoubleYoVariable secondSingularValueThreshold;
   private final DoubleYoVariable singularValueAlpha;
   private final DoubleYoVariable yoMinSingularValue;
   private final DoubleYoVariable[] yoSingularValues;
   private final DoubleYoVariable[] yoSingularValuesInverse;

   private final DenseMatrix64F tempV;
   
   public YoSolvePseudoInverseSVDWithDampedLeastSquaresNearSingularities(String namePrefix, int maxRows, int maxCols, YoVariableRegistry parentRegistry)
   {
      svd = DecompositionFactory.svd(maxRows, maxCols, true, true, true);
      tempV = new DenseMatrix64F(maxCols, maxCols);

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      mu = new DoubleYoVariable(namePrefix + "Mu", registry);
      firstSingularValueThreshold = new DoubleYoVariable(namePrefix + "FirstSingularValueThreshold", registry);
      secondSingularValueThreshold = new DoubleYoVariable(namePrefix + "SecondSingularValueThreshold", registry);
      singularValueAlpha = new DoubleYoVariable(namePrefix + "SingularValueAlpha", registry);
      yoMinSingularValue = new DoubleYoVariable(namePrefix + "MinSingularValue", registry);

      mu.set(0.003);
      firstSingularValueThreshold.set(5.0e-3);
      secondSingularValueThreshold.set(1.0e-5);

      yoSingularValues = new DoubleYoVariable[Math.max(maxRows, maxCols)];
      yoSingularValuesInverse = new DoubleYoVariable[Math.max(maxRows, maxCols)];

      for (int i = 0; i < yoSingularValues.length; i++)
      {
         yoSingularValues[i] = new DoubleYoVariable(namePrefix + "SingularValue_" + i, registry);
         yoSingularValues[i].set(Double.NaN);
      }

      for (int i = 0; i < yoSingularValuesInverse.length; i++)
      {
         yoSingularValuesInverse[i] = new DoubleYoVariable(namePrefix + "SingularValueInverse_" + i, registry);
         yoSingularValuesInverse[i].set(Double.NaN);
      }

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void setThresholds(double firstThreshold, double secondThreshold)
   {
      firstSingularValueThreshold.set(firstThreshold);
      secondSingularValueThreshold.set(secondThreshold);
   }

   public void setDampedLeastSquaresMu(double mu)
   {
      this.mu.set(mu);
   }

   private double alpha = 1.0;

   @Override
   public boolean setA(DenseMatrix64F A)
   {
      pseudoInverse.reshape(A.numCols, A.numRows, false);
      tempV.reshape(A.numCols, A.numRows, false);

      if (!svd.decompose(A))
         return false;

      DenseMatrix64F U_t = svd.getU(null, true);
      DenseMatrix64F V = svd.getV(tempV, false);
      double[] S = svd.getSingularValues();
      int N = Math.min(A.numRows, A.numCols);

      double minSingular = Double.POSITIVE_INFINITY;
      for (int i = 0; i < N; i++)
      {
         yoSingularValues[i].set(S[i]);

         if (S[i] < minSingular)
            minSingular = S[i];
      }

      yoMinSingularValue.set(minSingular);
      double deltaThresholds = firstSingularValueThreshold.getDoubleValue() - secondSingularValueThreshold.getDoubleValue();
      double muSquare = mu.getDoubleValue() * mu.getDoubleValue();

      alpha = 1.0;
      if (minSingular < secondSingularValueThreshold.getDoubleValue())
      {
         alpha = 0.0;
      }
      else if (minSingular < firstSingularValueThreshold.getDoubleValue())
      {
         alpha = (minSingular - secondSingularValueThreshold.getDoubleValue()) / deltaThresholds;
      }
      alpha = MathTools.clamp(alpha, 0.0, 1.0);
      alpha *= alpha;
      
      singularValueAlpha.set(alpha);

      // computer the pseudo inverse of A
      for (int i = 0; i < N; i++)
      {
         double s = S[i];

         if (minSingular <= secondSingularValueThreshold.getDoubleValue())
         {
            S[i] = s / (s * s + muSquare);
         }
         else
         {
            S[i] = alpha / s + (1.0 - alpha) * s / (s * s + muSquare);
         }

         yoSingularValuesInverse[i].set(S[i]);
      }

      // V*W
      for (int i = 0; i < V.numRows; i++)
      {
         int index = i * V.numCols;
         for (int j = 0; j < V.numCols; j++)
         {
            V.data[index++] *= S[j];
         }
      }

      // V*W*U^T
      CommonOps.mult(V, U_t, pseudoInverse);

      return true;
   }

   @Override
   public double quality()
   {
      return alpha;
   }

   @Override
   public void solve(DenseMatrix64F b, DenseMatrix64F x)
   {
      CommonOps.mult(pseudoInverse, b, x);
   }

   @Override
   public void invert(DenseMatrix64F A_inv)
   {
      A_inv.set(pseudoInverse);
   }

   @Override
   public boolean modifiesA()
   {
      return svd.inputModified();
   }

   @Override
   public boolean modifiesB()
   {
      return false;
   }

   @Override
   public SingularValueDecomposition<DenseMatrix64F> getDecomposition()
   {
      return svd;
   }
}
