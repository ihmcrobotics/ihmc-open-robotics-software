package us.ihmc.utilities.parameterOptimization.geneticAlgorithm;

import java.util.Comparator;

public class MaximizationIndividualComparator implements Comparator<GeneticAlgorithmIndividualToEvaluate>
{

   public int compare(GeneticAlgorithmIndividualToEvaluate individualOne, GeneticAlgorithmIndividualToEvaluate individualTwo)
   {
      double fitnessOne = individualOne.getFitness();
      double fitnessTwo = individualTwo.getFitness();
      
      if (fitnessOne > fitnessTwo)
         return -1;
      else if (fitnessOne < fitnessTwo)
         return 1;
      else
         return 0;
   }
}
