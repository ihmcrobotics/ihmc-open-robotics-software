package us.ihmc.robotics.numericalMethods;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
import us.ihmc.log.LogTools;

/**
 * The primary role of this solver is to calculate out closest manifold by searching configuration
 * spaces of the manifold.
 */
public class GradientDescentModule
{
   private static final boolean DEBUG = true;

   // internal
   private SingleQueryFunction function;
   private final int dimension;
   private final TDoubleArrayList initialInput;

   // result
   private boolean solved;
   private TDoubleArrayList optimalInput;
   private double optimalQuery;
   private double computationTime;

   // params
   private TDoubleArrayList inputUpperLimit;
   private TDoubleArrayList inputLowerLimit;
   private TDoubleArrayList stepSizes;
   private double deltaThreshold = 10E-10;
   private int maximumIterations = 1000;
   private double unboundedStepSize = -10.0; // TODO these signs are confusing...make everything positive
   private double stepSizeRatio = 0.5; // Step size
   private double perturbRatio = 0.5; // ratio on step size for gradient check
   private double reducingStepSizeRatio = 1.1; // Learning rate reduction
   private boolean verbose = false;

   public GradientDescentModule(SingleQueryFunction function, TDoubleArrayList initial)
   {
      this.function = function;
      this.dimension = initial.size();
      this.initialInput = new TDoubleArrayList();
      this.optimalInput = new TDoubleArrayList();
      this.inputUpperLimit = new TDoubleArrayList();
      this.inputLowerLimit = new TDoubleArrayList();
      this.stepSizes = new TDoubleArrayList();
      for (int i = 0; i < dimension; i++)
      {
         this.initialInput.add(initial.get(i));
         this.optimalInput.add(0.0);
         this.inputUpperLimit.add(Double.POSITIVE_INFINITY);
         this.inputLowerLimit.add(Double.NEGATIVE_INFINITY);
         this.stepSizes.add(stepSizeRatio);
      }
   }

   public void setVerbose(boolean verbose)
   {
      this.verbose = verbose;
   }

   public void redefineModule(SingleQueryFunction function)
   {
      this.function = function;
   }

   private void reduceStepSize()
   {
      for (int i = 0; i < dimension; i++)
      {
            stepSizes.set(i, stepSizes.get(i) / reducingStepSizeRatio);
      }
//      stepSizeRatio = stepSizeRatio / reducingStepSizeRatio;
   }

   /**
    * default value is 1000.
    */
   public void setMaximumIterations(int value)
   {
      maximumIterations = value;
   }

   public void setStepSizeRatio(double ratio)
   {
      stepSizeRatio = ratio;
   }

   public void setInputUpperLimit(TDoubleArrayList limit)
   {
      inputUpperLimit.clear();
      for (int i = 0; i < dimension; i++)
         inputUpperLimit.add(limit.get(i));
   }

   public void setInputLowerLimit(TDoubleArrayList limit)
   {
      inputLowerLimit.clear();
      for (int i = 0; i < dimension; i++)
         inputLowerLimit.add(limit.get(i));
   }

   /**
    * default value is 10E-10.
    */
   public void setConvergenceThreshold(double value)
   {
      deltaThreshold = value;
   }

   /**
    * default value is 1.
    */
   public void setUnboundedStepSize(double value)
   {
      if (value > 0)
         unboundedStepSize = -value;
      else
         unboundedStepSize = value;
   }

   /**
    * default value is 0.001.
    */
   public void setPerturbationSize(double value)
   {
      if (value > 0)
         perturbRatio = value;
      else
         perturbRatio = -value;
   }

   public void setReducingStepSizeRatio(double value)
   {
      reducingStepSizeRatio = value;
   }

   /**
    * The step size in each dimension depends on the search space size. If the search space is not bounded, set via a default value.
    */
   private static void setStepSizeFromLimits(TDoubleArrayList stepSizes, TDoubleArrayList inputUpperLimit, TDoubleArrayList inputLowerLimit, int dimension, double defaultStepSize, double stepSizeRatio)
   {
      stepSizes.clear();
      for (int i = 0; i < dimension; i++)
      {
         if (inputUpperLimit.get(i) == Double.POSITIVE_INFINITY || inputLowerLimit.get(i) == Double.NEGATIVE_INFINITY)
         {
            stepSizes.add(defaultStepSize);
         }
         else
         {
            double range = inputUpperLimit.get(i) - inputLowerLimit.get(i);
            stepSizes.add(-stepSizeRatio * range);
         }
      }
   }

   public int run()
   {
      long startTime = System.nanoTime();
      solved = false;

      int iteration = 0;
      TDoubleArrayList pastInput = new TDoubleArrayList();
      for (int i = 0; i < dimension; i++)
         pastInput.add(initialInput.get(i));

      optimalQuery = function.getQuery(pastInput);

      double pastQuery = 0;
      double newQuery = 0;

      // Set step sizes based on search space size
      setStepSizeFromLimits(stepSizes, inputUpperLimit, inputLowerLimit, dimension, unboundedStepSize, stepSizeRatio);

      // Start the run
      for (int i = 0; i < maximumIterations; i++)
      {
         long curTime = System.nanoTime();
         iteration++;
         pastQuery = optimalQuery;

         // Construct the gradient
         TDoubleArrayList gradient = new TDoubleArrayList();
         for (int j = 0; j < dimension; j++)
         {
            double tempSignForPerturb = 1.0;
            TDoubleArrayList perturbedInput = new TDoubleArrayList();
            for (int k = 0; k < dimension; k++)
            {
               perturbedInput.add(pastInput.get(k));
            }

            // If the gradient test goes out of bounds, check in the other direction
            double perturbSize = -perturbRatio * stepSizeRatio * stepSizes.get(j);
            if (perturbedInput.get(j) + perturbSize >= inputUpperLimit.get(j))
            {
               tempSignForPerturb = -1.0;
               if (verbose)
                  LogTools.info("current input is meeting with upper limit");
            }
            double tempInput = perturbedInput.get(j) + perturbSize * tempSignForPerturb;
            perturbedInput.set(j, MathTools.clamp(tempInput, inputLowerLimit.get(j), inputUpperLimit.get(j)));

            // Test f(x + dx)
            double perturbedQuery = function.getQuery(perturbedInput);
            // Gradient is (f(x+dx) - f(x)) / dx
            gradient.add((perturbedQuery - pastQuery) / (perturbSize * tempSignForPerturb));
         }
         // Normalize the gradient.
         double gradientNorm = 0;
         for (int j = 0; j < dimension; j++)
         {
            gradientNorm += gradient.get(j) * gradient.get(j);
         }
//         if (gradientNorm <= 1e-5) // TODO get good value
//         {
//            LogTools.info("=========================\nGradient norm too small\n===============================");
//            break;
//         }
         gradientNorm = Math.sqrt(gradientNorm);
//         LogTools.info("Gradient:");
         for (int j = 0; j < dimension; j++)
         {
            gradient.set(j, gradient.get(j) / gradientNorm);
//            LogTools.info(gradient.get(j));
         }

         // Construct next point to test
         optimalInput.clear();
         for (int j = 0; j < dimension; j++)
         {
            double input = pastInput.get(j) + gradient.get(j) * stepSizes.get(j);
            optimalInput.add(MathTools.clamp(input, inputLowerLimit.get(j), inputUpperLimit.get(j)));
         }

         newQuery = function.getQuery(optimalInput);
         if (verbose)
         {
            LogTools.info("cur Query " + pastQuery + " new Query " + newQuery);
            LogTools.info("Inputs: ");
            for (int j = 0; j < dimension; j++)
            {
               LogTools.info(pastInput.get(j));
            }
         }
         reduceStepSize();
         optimalQuery = newQuery;
         double delta = Math.abs((pastQuery - optimalQuery) / optimalQuery);

         if (verbose)
         {
            double iterationComputationTime = Conversions.nanosecondsToSeconds(System.nanoTime() - curTime);
            LogTools.info("iterations is " + i + " " + optimalQuery + " " + stepSizeRatio + " " + delta + " " + iterationComputationTime);
         }

         if (delta < deltaThreshold)
         {
            if (verbose) LogTools.info("End Early. Delta is: " + delta);
            break;
         }

         pastInput.clear();
         for (int j = 0; j < dimension; j++)
            pastInput.add(optimalInput.get(j));
      }

      LogTools.info("=========================\nFinished\n===============================");
      if (verbose)
      {
         LogTools.info("Optimal: ");
         for (int j = 0; j < dimension; j++)
         {
            LogTools.info(optimalInput.get(j));
         }
      }

      computationTime = Conversions.nanosecondsToSeconds(System.nanoTime() - startTime);
      return iteration;
   }

   public boolean isSolved()
   {
      return solved;
   }

   public TDoubleArrayList getOptimalInput()
   {
      return optimalInput;
   }

   public double getOptimalQuery()
   {
      return optimalQuery;
   }

   public double getComputationTime()
   {
      return computationTime;
   }
}