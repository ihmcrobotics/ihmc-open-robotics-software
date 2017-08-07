package us.ihmc.utilities.parameterOptimization.geneticAlgorithm;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.utilities.parameterOptimization.IndividualToEvaluate;
import us.ihmc.utilities.parameterOptimization.OptimizationProblem;

public class GeneticAlgorithmTest
{
   private static final boolean SHOW_GUI = false;

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void testOptimizeExampleIndividualOne()
   {
      int populationSize = 500;
      double crossoverRate = 0.8;
      double mutationRate = 0.01;
      String name = "GeneticAlgorithmTestOne";
      
      Random random = new Random(1776L);
      
      PopulationParameters populationParameters = new PopulationParameters(name, random, populationSize);
      populationParameters.setSeedIndividualsToCopyIntoFirstPopulation(populationSize/2, 0.01);
      
      GeneticAlgorithm geneticAlgorithm = new GeneticAlgorithm(populationParameters, crossoverRate, mutationRate);
      
      if (SHOW_GUI) geneticAlgorithm.createGUI();

      IndividualToEvaluate seedIndividual = new ExampleIndividualToEvaluateOne();
      boolean maximize = true;
      double cutoffFitness = 15.0; 
      int maximumNumberOfIndividualsToEvaluate = 100000;
      
      OptimizationProblem optimizationProblem = new OptimizationProblem(seedIndividual, maximize, cutoffFitness, maximumNumberOfIndividualsToEvaluate);
      IndividualToEvaluate optimalIndividual = geneticAlgorithm.optimize(optimizationProblem);
   
      double fitness = optimalIndividual.getFitness();
      
      if (SHOW_GUI)
      {
         sleepForever();
      }
      
      assertTrue(fitness >= cutoffFitness);

   }
   
   
//   @Test(timeout=300000)
   public void testOptimizeExampleIndividualTwo()
   {
      int populationSize = 50;
      double crossoverRate = 0.6;
      double mutationRate = 0.02;
      String name = "GeneticAlgorithmTestTwo";
      Random random = new Random(1984L);
      
      PopulationParameters populationParameters = new PopulationParameters(name, random, populationSize);
      
      GeneticAlgorithm geneticAlgorithm = new GeneticAlgorithm(populationParameters, crossoverRate, mutationRate);
            
      if (SHOW_GUI) geneticAlgorithm.createGUI();

      IndividualToEvaluate seedIndividual = new ExampleIndividualToEvaluateTwo();
      boolean maximize = true;
      double cutoffFitness = 0.999;
      int maximumNumberOfIndividualsToEvaluate = 2500;
      
      OptimizationProblem optimizationProblem = new OptimizationProblem(seedIndividual, maximize, cutoffFitness, maximumNumberOfIndividualsToEvaluate);
      IndividualToEvaluate optimalIndividual = geneticAlgorithm.optimize(optimizationProblem);
    
      
      double fitness = optimalIndividual.getFitness();
      assertTrue(fitness >= cutoffFitness);
      
      if (SHOW_GUI)
      {
         sleepForever();
      }
      
   }

   private void sleepForever()
   {
      while(true)
      {
         try
         {
            Thread.sleep(1000);
         } 
         catch (InterruptedException e)
         {
         }
      }
      
   }

}
