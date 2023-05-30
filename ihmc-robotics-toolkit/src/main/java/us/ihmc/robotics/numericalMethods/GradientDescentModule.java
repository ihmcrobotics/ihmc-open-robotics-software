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
   private static final boolean DEBUG = false;

   // internal
   private SingleQueryFunction function;
   private final int dimension;
   private final TDoubleArrayList initialInput;
   private long startTime;
   private double pastQuery;
   private double newQuery;
   private TDoubleArrayList pastInput;

   // result
   private boolean solved;
   private TDoubleArrayList optimalInput;
   private double optimalQuery;
   private double computationTime;
   private int iteration = 0;

   // params
   private TDoubleArrayList inputUpperLimit;
   private TDoubleArrayList inputLowerLimit;
   private double deltaThreshold = 10E-10;
   private int maximumIterations = 1000;
   private double alpha = -10;
   private double perturb = 0.001;
   private int reducingStepSizeRatio = 2;

   public GradientDescentModule(SingleQueryFunction function, TDoubleArrayList initial)
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

   public void redefineModule(SingleQueryFunction function)
   {
      this.function = function;
   }

   private void reduceStepSize()
   {
      alpha = alpha / reducingStepSizeRatio;
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
    * default value is 1.
    */
   public void setStepSize(double value)
   {
      if (value > 0)
         alpha = -value;
      else
         alpha = value;
   }

   /**
    * default value is 0.001.
    */
   public void setPerturbationSize(double value)
   {
      if (value > 0)
         perturb = value;
      else
         perturb = -value;
   }

   public void setReducingStepSizeRatio(int value)
   {
      reducingStepSizeRatio = value;
   }

   public void initializeOptimizer()
   {
      startTime = System.nanoTime();
      solved = false;

      iteration = 0;
      pastInput = new TDoubleArrayList();
      for (int i = 0; i < dimension; i++)
         pastInput.add(initialInput.get(i));

      optimalQuery = function.getQuery(pastInput);

      pastQuery = 0;
      newQuery = 0;
   }

   /**
    *
    * @return whether DeltaThreshold has been reached
    */
   public boolean takeOptimizationStep()
   {
      long curTime = System.nanoTime();
      iteration++;
      pastQuery = optimalQuery;

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

         double perturbedQuery = function.getQuery(perturbedInput);

         gradient.add((perturbedQuery - pastQuery) / (perturb * tempSignForPerturb));
      }

      optimalInput.clear();
      for (int j = 0; j < dimension; j++)
      {
         double input = pastInput.get(j) + gradient.get(j) * alpha;
         optimalInput.add(MathTools.clamp(input, inputLowerLimit.get(j), inputUpperLimit.get(j)));
      }

      newQuery = function.getQuery(optimalInput);
      if (DEBUG)
         LogTools.debug("cur Query " + pastQuery + " new Query " + newQuery);

      if (newQuery > pastQuery)
      {
         reduceStepSize();
         optimalInput.clear();
         for (int j = 0; j < dimension; j++)
            optimalInput.add(pastInput.get(j));
      }
      else
      {
         optimalQuery = newQuery;
         double delta = Math.abs((pastQuery - optimalQuery) / optimalQuery);

         if (DEBUG)
         {
            double iterationComputationTime = Conversions.nanosecondsToSeconds(System.nanoTime() - curTime);
            LogTools.debug("iterations is " + iteration + " " + optimalQuery + " " + alpha + " " + delta + " " + iterationComputationTime);
         }
         if (delta < deltaThreshold)
            return true;
      }

      pastInput.clear();
      for (int j = 0; j < dimension; j++)
         pastInput.add(optimalInput.get(j));

      return false;

   }

   public boolean optimizerIsFinished()
   {
      return (iteration >= maximumIterations);
   }



   public int run()
   {
      initializeOptimizer();

      for (int i = 0; i < maximumIterations; i++)
      {
         boolean completed = takeOptimizationStep();
         if (completed)
            break;
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