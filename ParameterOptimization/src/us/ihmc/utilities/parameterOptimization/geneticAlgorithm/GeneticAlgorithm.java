package us.ihmc.utilities.parameterOptimization.geneticAlgorithm;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

import us.ihmc.utilities.parameterOptimization.IndividualToEvaluate;
import us.ihmc.utilities.parameterOptimization.OptimizationProblem;
import us.ihmc.utilities.parameterOptimization.ParameterOptimizer;
import us.ihmc.utilities.parameterOptimization.geneticAlgorithm.gui.GeneticAlgorithmGUI;

/**
 * <p>Title: Genetic Algorithm Library </p>
 *
 * <p>Description: General Purpose Genetic Algorithm Library </p>
 *
 * <p>Copyright: Copyright (c) 2003-2005 Jerry Pratt, IHMC </p>
 *
 * <p>Company: Institute for Human and Machine Cognition.
 * 40 South Alcaniz Street
 * Pensacola, FL 32502 </p>
 *
 * @author Jerry Pratt and Jim Warrenfeltz, jpratt@ihmc.us
 * @version 1.0
 */

public class GeneticAlgorithm implements ParameterOptimizer
{
   private ArrayList<Population> populations = new ArrayList<Population>();
   private String name;
   private double crossoverRate, mutationRate;
   private int populationSize;
   private ArrayList<GeneticAlgorithmChangedListener> listeners = new ArrayList<GeneticAlgorithmChangedListener>();

   public GeneticAlgorithm(IndividualToEvaluate individual, int populationSize, double crossoverRate, double mutationRate, String name)
   {
      // crossoverRate is the probability that two parents will produce children, as opposed to enter next population themselves.
      // mutation rate is the probability for each bit to be flipped.

      
      GeneticAlgorithmIndividualToEvaluate geneticAlgorithmIndividualToEvaluate = new GeneticAlgorithmIndividualToEvaluate(individual);
      
      this.populationSize = populationSize;
      this.crossoverRate = crossoverRate;
      this.mutationRate = mutationRate;

      this.name = name;

      Population population = new Population(populationSize, geneticAlgorithmIndividualToEvaluate, name, 0);
      populations.add(population);
   }

   public GeneticAlgorithm(Population population, double crossoverRate, double mutationRate, String name)
   {
      // crossoverRate is the probability that two parents will produce children, as opposed to enter next population themselves.
      // mutation rate is the probability for each bit to be flipped.

      this.populationSize = population.getNumberOfIndividuals();
      this.crossoverRate = crossoverRate;
      this.mutationRate = mutationRate;

      this.name = name;

      populations.add(population);
   }

   public String getName()
   {
      return name;
   }

   public void createGUI()
   {
      GeneticAlgorithmGUI gui = new GeneticAlgorithmGUI(this);
   }

   public void evolve(int generations)
   {
      evolve(generations, null);
   }

   public void evolve(int generations, String filename)
   {
      for (int i = 0; i < generations; i++)
      {
         evolveOneGeneration();
         notifyGeneticAlgorithmChangedListeners();

         if (filename != null)
         {
            Population latestPop = (Population) populations.get(populations.size() - 2);
            latestPop.save(filename + latestPop.getPopulationNumber());
         }

         // System.out.println(getFittestIndividual());
      }

      Population finalPopulation = (Population) populations.get(populations.size() - 1);
      finalPopulation.evaluateAndSortByFitness();
      notifyGeneticAlgorithmChangedListeners();

      finalPopulation.save(filename + finalPopulation.getPopulationNumber());
   }

   public void evolveToFitness(double fitnessCutoff)
   {
      double peakFitness = getFittestIndividual().getFitness();
      while (peakFitness < fitnessCutoff)
      {
         evolveOneGeneration();

         // System.out.println(getFittestIndividual());
         peakFitness = getFittestIndividual().getFitness();
      }

      notifyGeneticAlgorithmChangedListeners();
   }

   public GeneticAlgorithmIndividualToEvaluate getFittestIndividual()
   {
      Population pop = (Population) populations.get(populations.size() - 1);

      return pop.getFittestIndividual();
   }

   public void setCrossoverRate(double xorate)
   {
      crossoverRate = xorate;
      notifyGeneticAlgorithmChangedListeners();
   }

   public void setMutationRate(double mrate)
   {
      mutationRate = mrate;
      notifyGeneticAlgorithmChangedListeners();
   }

   public double getCrossoverRate()
   {
      return crossoverRate;
   }

   public double getMutatationRate()
   {
      return mutationRate;
   }

   public int getPopulationSize()
   {
      return populationSize;
   }

   public int getNumberOfPopulations()
   {
      return populations.size();
   }

   public Population getPopulation(int index)
   {
      return (Population) populations.get(index);
   }

   public Population[] getPopulations()
   {
      Population[] ret = new Population[populations.size()];

      populations.toArray(ret);

      return ret;
   }

   public Population evolveOneGeneration()
   {
      Population pop = (Population) populations.get(populations.size() - 1);
      Population ret = pop.breed(crossoverRate, mutationRate);
      populations.add(ret);

      notifyGeneticAlgorithmChangedListeners();

      return ret;
   }

   public void addGeneticAlgorithmChangedListener(GeneticAlgorithmChangedListener listener)
   {
      listeners.add(listener);
   }

   public void notifyGeneticAlgorithmChangedListeners()
   {
      for (int i = 0; i < listeners.size(); i++)
      {
         GeneticAlgorithmChangedListener listener = (GeneticAlgorithmChangedListener) listeners.get(i);
         listener.geneticAlgorithmChanged();
      }
   }

   public void save(String filename)
   {
      if (!filename.endsWith(".ga"))
      {
         filename = filename + ".ga";
      }

      // File file = new File(filename);

      try
      {
         PrintWriter writer = new PrintWriter(new FileOutputStream(filename));

         for (int popIndex = 0; popIndex < populations.size(); popIndex++)
         {
            Population pop = (Population) populations.get(popIndex);

            writer.println("Population: " + pop.getPopulationNumber());
            writer.println("");

            GeneticAlgorithmIndividualToEvaluate[] individuals = pop.getAllIndividuals();
            for (int indIndex = 0; indIndex < individuals.length; indIndex++)
            {
               GeneticAlgorithmIndividualToEvaluate individual = individuals[indIndex];
               writer.println(individual);
               writer.println("");
            }

            writer.println("+++++++++++++++++++++++++++++++++++++++");

         }

         writer.close();

      }
      catch (IOException e)
      {
         System.err.println("File IO Error in GeneticAlgorithm.save()");
      }

   }

   public IndividualToEvaluate optimize(OptimizationProblem optimizationProblem)
   {
//      IndividualToEvaluate seedIndividualToEvaluate = optimizationProblem.getSeedIndividualToEvaluate();
      
      this.evolveToFitness(optimizationProblem.getCutoffFitness());
      
      GeneticAlgorithmIndividualToEvaluate fittestIndividual = getFittestIndividual();
      return fittestIndividual.getIndividualToEvaluate();
   }

   public void attachListener()
   {
      throw new RuntimeException("Not implemented yet!");      
   }

   /*
    * public String toString()
    *  {
    * return "";
    *  }
    */


}
