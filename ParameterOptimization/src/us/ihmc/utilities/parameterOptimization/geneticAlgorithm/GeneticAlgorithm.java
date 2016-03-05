package us.ihmc.utilities.parameterOptimization.geneticAlgorithm;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Random;

import us.ihmc.utilities.parameterOptimization.EvaluatedIndividualListener;
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
   private ArrayList<EvaluatedIndividualListener> evaluatedIndividualListeners;
   
   private final ArrayList<Population> populations = new ArrayList<Population>();
   private double crossoverRate, mutationRate;
   
   private PopulationParameters populationParameters;
   
   private final ArrayList<GeneticAlgorithmChangedListener> listeners = new ArrayList<GeneticAlgorithmChangedListener>();

//   public GeneticAlgorithm(Random random, int populationSize, double crossoverRate, double mutationRate, String name)
//   {
//      // crossoverRate is the probability that two parents will produce children, as opposed to enter next population themselves.
//      // mutation rate is the probability for each bit to be flipped.      
//      
//      this.populationSize = populationSize;
//      this.random = random;
//      
//      this.crossoverRate = crossoverRate;
//      this.mutationRate = mutationRate;
//
//      this.name = name;
//   }
   
   public GeneticAlgorithm(PopulationParameters populationParameters, double crossoverRate, double mutationRate)
   {
      this.populationParameters = populationParameters;
      
      this.crossoverRate = crossoverRate;
      this.mutationRate = mutationRate;
      
      if (populationParameters.getSeedIndividualToEvaluate() != null)
      {
         Population population = new Population(populationParameters, 0);
         populations.add(population);
      }
   }

   public GeneticAlgorithm(Random random, Population population, double crossoverRate, double mutationRate, String name)
   {
      // crossoverRate is the probability that two parents will produce children, as opposed to enter next population themselves.
      // mutation rate is the probability for each bit to be flipped.
      this.populationParameters = new PopulationParameters(name, random, population.getNumberOfIndividuals());
      
      this.crossoverRate = crossoverRate;
      this.mutationRate = mutationRate;
      
      populations.add(population);
   }

   public String getName()
   {
      return populationParameters.getName();
   }
   
   public void createGUI()
   {
      new GeneticAlgorithmGUI(this);
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

   public void evolveToFitness(boolean maximize, double fitnessCutoff, int maximumNumberOfIndividualsToEvaluate)
   {
      double peakFitness = getFittestIndividual().getFitness();
      
      
      while (((maximize && (peakFitness < fitnessCutoff)) || (!maximize && (peakFitness > fitnessCutoff))) && 
            (this.getNumberOfIndividuals() < maximumNumberOfIndividualsToEvaluate))
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

   public void setCrossoverRate(double crossoverRate)
   {
      this.crossoverRate = crossoverRate;
      notifyGeneticAlgorithmChangedListeners();
   }

   public void setMutationRate(double mutationRate)
   {
      this.mutationRate = mutationRate;
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
      return populationParameters.getPopulationSize();
   }

   public int getNumberOfPopulations()
   {
      return populations.size();
   }
   
   public int getNumberOfIndividuals()
   {
      return getPopulationSize() * getNumberOfPopulations();
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
      // The population holds the seed individuals and the comparator. If there is one already, then just evolve that.
      // If not, then need to create one based on the optimization problem:
      if (this.populations.isEmpty())
      {
         System.out.println("Populations empty. Creating new one");
         
         IndividualToEvaluate seedIndividualToEvaluate = optimizationProblem.getSeedIndividualToEvaluate();
         
         boolean maximize = optimizationProblem.getMaximize();
         
         Comparator<GeneticAlgorithmIndividualToEvaluate> comparator;
         if (maximize)
         {
            comparator = new MaximizationIndividualComparator();
         }
         else
         {
            comparator = new MinimizationIndividualComparator();
         }
                  
         populationParameters.setComparator(comparator);
         populationParameters.setSeedIndividualToEvaluate(seedIndividualToEvaluate);
         
         Population population = new Population(populationParameters, 0);
                  
         this.populations.add(population);
      }
      
      
      boolean maximize = optimizationProblem.getMaximize();
      
      this.evolveToFitness(maximize, optimizationProblem.getCutoffFitness(), optimizationProblem.getMaximumNumberOfIndividualsToEvaluate());
      
      GeneticAlgorithmIndividualToEvaluate fittestIndividual = getFittestIndividual();
      return fittestIndividual.getIndividualToEvaluate();
   }
   
   public void attachEvaluatedIndividualListener(EvaluatedIndividualListener listener)
   {
      if (evaluatedIndividualListeners == null) evaluatedIndividualListeners = new ArrayList<EvaluatedIndividualListener>();
      
      evaluatedIndividualListeners.add(listener);
   }


}
