package us.ihmc.utilities.parameterOptimization.geneticAlgorithm;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Comparator;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class PopulationTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.6)
	@Test(timeout = 30000)
   public void testPopulation()
   {
      Random random = new Random(1776L);
      
      ExampleIndividualToEvaluateOne seedIndividualToEvaluate = new ExampleIndividualToEvaluateOne();
//      GeneticAlgorithmIndividualToEvaluate testIndividualToEvaluate = GeneticAlgorithmIndividualToEvaluate.makeRandomIndividual(seedIndividualToEvaluate); 

//      System.out.println("   Creating Population");
      int populationSize = 500;
      Comparator<GeneticAlgorithmIndividualToEvaluate> comparator = new MaximizationIndividualComparator();

      
      PopulationParameters populationParameters = new PopulationParameters("test", random, populationSize);
      populationParameters.setComparator(comparator);
      populationParameters.setSeedIndividualToEvaluate(seedIndividualToEvaluate);
      
      Population testerPop = new Population(populationParameters, 0);
      
      assertFalse(testerPop.allIndividualsEvaluated());
      testerPop.evaluateAndSortByFitness();
      assertTrue(testerPop.allIndividualsEvaluated());
      
      assertEquals(populationSize, testerPop.getNumberOfIndividuals());
      
//      System.out.println("   Breeding Population");
      Population newPop = testerPop.breed(0.20, 0.02);
//      System.out.println("   Done Breeding Population");

      // Breed a bunch of populations and make sure that it gets optimized:
      
      double bestFitness = 0.0;
      
      for (int i=0; i<200; i++)
      {
//         System.out.println("Generation: " + gen);

         newPop = newPop.breed(0.60, 0.002);
         assertEquals(populationSize, newPop.getNumberOfIndividuals());
         
         GeneticAlgorithmIndividualToEvaluate[] allIndividuals = newPop.getAllIndividuals();
         assertEquals(populationSize, allIndividuals.length);
         
         double totalFitness = 0.0;
         for (int j=0; j<populationSize; j++)
         {
            GeneticAlgorithmIndividualToEvaluate individual = newPop.getIndividual(j);
            assertEquals(allIndividuals[j].getName(), individual.getName());
            totalFitness += individual.getFitness();
         }
         double averageFitness = totalFitness / ((double) populationSize);
         assertEquals(averageFitness, newPop.getAverageFitness(), 1e-7);
         
//         System.out.println(newPop);
         
         GeneticAlgorithmIndividualToEvaluate fittestIndividual = newPop.getFittestIndividual();
         
//         System.out.println(fittestIndividual);
         
         double maximumFitness = fittestIndividual.getFitness();
         if (maximumFitness > bestFitness) bestFitness = maximumFitness;
         
         assertEquals(maximumFitness, newPop.getMaximumFitness(), 1e-7);
         
         GeneticAlgorithmIndividualToEvaluate leastFitIndividual = newPop.getLeastFitIndividual();

         
         double minimumFitness = leastFitIndividual.getFitness();
         assertEquals(minimumFitness, newPop.getMinimumFitness(), 1e-7);

         assertTrue(maximumFitness >= minimumFitness);
      }
      
//      System.out.println(bestFitness);
      assertTrue("bestFitness = " + bestFitness, bestFitness > 13.0);
   }

}
