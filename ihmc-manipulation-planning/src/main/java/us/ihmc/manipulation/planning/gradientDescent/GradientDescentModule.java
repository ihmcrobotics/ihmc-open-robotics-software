package us.ihmc.manipulation.planning.gradientDescent;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;

/**
 * The primary role of this solver is to calculate out closest manifold by searching configuration spaces of the manifold. 
 */
public class GradientDescentModule
{
   private final boolean DEBUG = false;
   
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
   private double deltaThreshold = 10E-10;
   private int maximumIterations = 1000;
   private double alpha = -10;
   private double perturb = 0.001;
   
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

   private void reduceStepSize()
   {
      alpha = alpha * 0.1;
   }

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
   
   public void setConvergenceThreshold(double value)
   {
      deltaThreshold = value;
   }

   public int run()
   {
      long startTime = System.nanoTime();
      solved = false;
      optimalQuery = Double.MAX_VALUE;

      int iteration = 0;
      TDoubleArrayList pastInput = new TDoubleArrayList();
      for (int i = 0; i < dimension; i++)
         pastInput.add(initialInput.get(i));
      for (int i = 0; i < maximumIterations; i++)
      {
         iteration++;
         double pastQuery = function.getQuery(pastInput);

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
                  System.out.println("current input is meeting with upper limit");
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

         optimalQuery = function.getQuery(optimalInput);

         if (optimalQuery > pastQuery)
         {
            reduceStepSize();
            optimalInput.clear();
            for (int j = 0; j < dimension; j++)
               optimalInput.add(pastInput.get(j));
         }

         double delta = Math.abs((pastQuery - optimalQuery) / optimalQuery);

         if (DEBUG)
            System.out.println("GradientDescentModule " + i + " " + optimalQuery + " " + optimalInput.get(0) + " " + gradient.get(0) + " " + delta);

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