package us.ihmc.robotics.numericalMethods;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
import us.ihmc.log.LogTools;

import java.util.function.ToDoubleFunction;

/**
 * The primary role of this solver is to calculate out closest manifold by searching configuration
 * spaces of the manifold.
 */
public class GradientDescentModule
{
   private static final boolean DEBUG = false;

   // internal
   private ToDoubleFunction<TDoubleArrayList> function;
   private final int dimension;
   private final TDoubleArrayList initialInput;

   // result
   private boolean solved;
   private final TDoubleArrayList optimalInput;
   private double optimalQuery;
   private double computationTime;

   // params
   private final TDoubleArrayList inputUpperLimit;
   private final TDoubleArrayList inputLowerLimit;
   private double deltaThreshold = 10E-10;
   private int maximumIterations = 1000;
   private double learningRate = -1.0;
   private double minLearningRate = -1e-6;
   private double learningRateToUse = learningRate;
   private double perturb = 0.001;
   private double reducingLearningRateRatio = 1.1;

   public GradientDescentModule(ToDoubleFunction<TDoubleArrayList> function, TDoubleArrayList initial)
   {
      this.function = function;
      this.dimension = initial.size();
      this.initialInput = new TDoubleArrayList();
      this.optimalInput = new TDoubleArrayList();
      this.inputUpperLimit = new TDoubleArrayList();
      this.inputLowerLimit = new TDoubleArrayList();
      for (int i = 0; i < dimension; i++)
      {
         this.initialInput.add(initial.get(i));
         this.optimalInput.add(0.0);
         this.inputUpperLimit.add(Double.POSITIVE_INFINITY);
         this.inputLowerLimit.add(Double.NEGATIVE_INFINITY);
      }
   }

   public void redefineModule(ToDoubleFunction<TDoubleArrayList> function)
   {
      this.function = function;
   }

   private void reduceLearningRate()
   {
      learningRateToUse = Math.min(learningRateToUse / reducingLearningRateRatio, minLearningRate);
   }

   /**
    * default value is 1000.
    */
   public void setMaximumIterations(int value)
   {
      maximumIterations = value;
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
    * This is the learning rate applied to the gradient during the update. The default value is -1.0.
    */
   public void setLearningRate(double value)
   {
      learningRate = -Math.abs(value);
   }

   /**
    * Sets the minimum learning rate that can be applied after the learning rate reduction. The learning rate is iteratively reduced on each iteration by
    * {@link #reducingLearningRateRatio}. This bounds how much it can be reduced by..
    */
   public void setMinimumLearningRate(double value)
   {
      minLearningRate = -Math.abs(value);
   }

   /**
    * This is the perturbation size used to numerically compute the gradient.
    * default value is 0.001.
    */
   public void setPerturbationSize(double value)
   {
      perturb = Math.abs(value);
   }

   public void setReducingLearningRateRatio(double value)
   {
      reducingLearningRateRatio = value;
   }

   public int run()
   {
      long startTime = System.nanoTime();
      solved = false;

      int iteration = 0;
      TDoubleArrayList pastInput = new TDoubleArrayList();
      for (int i = 0; i < dimension; i++)
         pastInput.add(initialInput.get(i));

      optimalQuery = function.applyAsDouble(pastInput);
      // reinitialize the learning rate to its original value, in case it was changed using the reducing learning rate
      learningRateToUse = learningRate;

      double pastQuery = 0;
      double newQuery = 0;

      for (int i = 0; i < maximumIterations; i++)
      {
         long curTime = System.nanoTime();
         iteration++;
         pastQuery = optimalQuery;

         // Construct the gradient
         double tempSignForPerturb = 1.0;
         TDoubleArrayList gradient = new TDoubleArrayList();
         for (int j = 0; j < dimension; j++)
         {
            TDoubleArrayList perturbedInput = new TDoubleArrayList();
            for (int k = 0; k < dimension; k++)
               perturbedInput.add(pastInput.get(k));

            if (perturbedInput.get(j) == inputUpperLimit.get(j))
            {
               tempSignForPerturb = -1.0;
               if (DEBUG)
                  LogTools.debug("current input is meeting with upper limit");
            }

            double tempInput = perturbedInput.get(j) + perturb * tempSignForPerturb;

            perturbedInput.replace(j, MathTools.clamp(tempInput, inputLowerLimit.get(j), inputUpperLimit.get(j)));

            double perturbedQuery = function.applyAsDouble(perturbedInput);

            gradient.add((perturbedQuery - pastQuery) / (perturb * tempSignForPerturb));
         }
         // Normalize the gradient
         double gradientNorm = 0;
         for (int j = 0; j < dimension; j++)
         {
            gradientNorm += gradient.get(j) * gradient.get(j);
         }
         gradientNorm = Math.sqrt(gradientNorm);
         for (int j = 0; j < dimension; j++)
         {
            gradient.set(j, gradient.get(j) / gradientNorm);
         }

         // Construct next point to test
         optimalInput.clear();
         for (int j = 0; j < dimension; j++)
         {
            double input = pastInput.get(j) + gradient.get(j) * learningRateToUse;
            optimalInput.add(MathTools.clamp(input, inputLowerLimit.get(j), inputUpperLimit.get(j)));
         }

         newQuery = function.applyAsDouble(optimalInput);
         if (DEBUG)
            LogTools.debug("cur Query " + pastQuery + " new Query " + newQuery);

         reduceLearningRate();
         optimalQuery = newQuery;
         double delta = Math.abs((pastQuery - optimalQuery) / optimalQuery);

         if (DEBUG)
         {
            double iterationComputationTime = Conversions.nanosecondsToSeconds(System.nanoTime() - curTime);
            LogTools.debug("iterations is " + i + " " + optimalQuery + " " + learningRateToUse + " " + delta + " " + iterationComputationTime);
         }

         if (delta < deltaThreshold)
            break;


         pastInput.clear();
         for (int j = 0; j < dimension; j++)
            pastInput.add(optimalInput.get(j));
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