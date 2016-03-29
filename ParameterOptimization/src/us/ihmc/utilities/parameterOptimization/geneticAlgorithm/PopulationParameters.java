package us.ihmc.utilities.parameterOptimization.geneticAlgorithm;

import java.util.Comparator;
import java.util.Random;

import us.ihmc.utilities.parameterOptimization.IndividualToEvaluate;

public class PopulationParameters
{
   private final String name;
   private final Random random;

   private final int populationSize;

   private Comparator<GeneticAlgorithmIndividualToEvaluate> comparator;
   private IndividualToEvaluate seedIndividualToEvaluate;
   
   private int numberOfSeedIndividualsToCopyIntoFirstPopulation = 0;
   private double mutationRateForCopiedIndividuals = 0.0;
   
   public PopulationParameters(String name, Random random, int populationSize)
   {
      this.name = name;
      this.random = random;
      this.populationSize = populationSize;
   }

   public int getPopulationSize()
   {
      return populationSize;
   }

   public String getName()
   {
      return name;
   }

   public Comparator<GeneticAlgorithmIndividualToEvaluate> getComparator()
   {
      return comparator;
   }

   public void setComparator(Comparator<GeneticAlgorithmIndividualToEvaluate> comparator)
   {
      this.comparator = comparator;
   }

   public IndividualToEvaluate getSeedIndividualToEvaluate()
   {
      return seedIndividualToEvaluate;
   }

   public void setSeedIndividualToEvaluate(IndividualToEvaluate seedIndividualToEvaluate)
   {
      this.seedIndividualToEvaluate = seedIndividualToEvaluate;
   }

   public int getNumberOfSeedIndividualsToCopyIntoFirstPopulation()
   {
      return numberOfSeedIndividualsToCopyIntoFirstPopulation;
   }
   
   public double getMutationRateForCopiedIndividuals()
   {
      return mutationRateForCopiedIndividuals;
   }

   public void setSeedIndividualsToCopyIntoFirstPopulation(int numberOfSeedIndividualsToCopyIntoFirstPopulation, double mutationRate)
   {
      this.numberOfSeedIndividualsToCopyIntoFirstPopulation = numberOfSeedIndividualsToCopyIntoFirstPopulation;
      this.mutationRateForCopiedIndividuals = mutationRate;
   }

   public Random getRandom()
   {
      return random;
   }
  

}
