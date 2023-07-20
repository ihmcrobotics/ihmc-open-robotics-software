package us.ihmc.robotics.optimization;

import gnu.trove.list.array.TDoubleArrayList;
import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;
import us.ihmc.robotics.numericalMethods.SingleQueryFunction;

/**
 * Gradient descent with finite difference
 */
public class WrappedGradientDescent implements Optimizer
{
   private GradientDescentModule gradientDescentModule;
   private CostFunction costFunction;

   private final DMatrixD1 vectorInputToCostFunction = new DMatrixRMaj();
   private final DMatrixD1 optimalInput = new DMatrixRMaj();
   private double stepSize = 10.0;
   private double learningRate = 0.9;
   private int maxIterations;
   private TDoubleArrayList lowerBound = null;
   private TDoubleArrayList upperBound = null;
   private boolean verbose = false;

   public WrappedGradientDescent()
   {

   }

   @Override
   public void setCostFunction(CostFunction costFunction)
   {
      this.costFunction = costFunction;
   }

   private SingleQueryFunction createUnwrappedCostFunction(CostFunction costFunction)
   {
      return new SingleQueryFunction()
      {
         @Override
         public double getQuery(TDoubleArrayList values)
         {
            // convert TDoubleArrayList to DMatrixRMaj
            convertArrayToMatrix(vectorInputToCostFunction, values);
            return costFunction.calculate(vectorInputToCostFunction);
         }
      };
   }

   private static void convertArrayToMatrix(DMatrixD1 vector, TDoubleArrayList list)
   {
      vector.setData(list.toArray());
      vector.reshape(list.size(), 1);
   }

   private static void convertMatrixToArray(DMatrixD1 vector, TDoubleArrayList list)
   {
      list.reset();
      list.addAll(vector.data);
   }

   public void setInitialStepSize(double stepSize)
   {
      this.stepSize = stepSize;
   }

   public void setLearningRate(double learningRate)
   {
      this.learningRate = learningRate;
   }

   public void setMaxIterations(int maxIterations) {this.maxIterations = maxIterations;}

   public void setVerbose(boolean verbose)
   {
      this.verbose = verbose;
   }

   /**
    * Not implemented
    * @return
    */
   @Override
   public DMatrixD1 stepOneIteration()
   {
      return null;
   }

   @Override
   public DMatrixD1 optimize(DMatrixD1 initial)
   {
      TDoubleArrayList initialArray = new TDoubleArrayList();
      convertMatrixToArray(initial, initialArray);
      gradientDescentModule = new GradientDescentModule(createUnwrappedCostFunction(costFunction), initialArray);
      gradientDescentModule.setUnboundedStepSize(stepSize);
      gradientDescentModule.setReducingStepSizeRatio(1.0/learningRate);
      gradientDescentModule.setMaximumIterations(maxIterations);
      gradientDescentModule.setInputLowerLimit(lowerBound);
      gradientDescentModule.setInputUpperLimit(upperBound);
      gradientDescentModule.setVerbose(verbose);

      gradientDescentModule.run();
      return getOptimalParameters();
   }

   @Override
   public DMatrixD1 getOptimalParameters()
   {
      convertArrayToMatrix(optimalInput, gradientDescentModule.getOptimalInput());
      return optimalInput;
   }

   @Override
   public double getOptimumCost()
   {
      return gradientDescentModule.getOptimalQuery();
   }

   @Override
   public void setRealDomain(RealDomainBounds[] bounds)
   {
      // assert that bounds is correct size

      lowerBound = new TDoubleArrayList();
      upperBound = new TDoubleArrayList();
      for (RealDomainBounds bound : bounds)
      {
         lowerBound.add(bound.min());
         upperBound.add(bound.max());
      }
   }
}
