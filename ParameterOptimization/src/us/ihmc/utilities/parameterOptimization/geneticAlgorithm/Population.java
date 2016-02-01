package us.ihmc.utilities.parameterOptimization.geneticAlgorithm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Random;

import us.ihmc.utilities.parameterOptimization.EvaluatedIndividualListener;
import us.ihmc.utilities.parameterOptimization.IndividualToEvaluate;

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

public class Population
{
   private ArrayList<EvaluatedIndividualListener> evaluatedIndividualListeners;

   private final Random random;
   
   private final GeneticAlgorithmIndividualToEvaluate[] generation;
   private final String popName;
   private final int popNumber;

   private final Comparator<GeneticAlgorithmIndividualToEvaluate> comparator;
   
   private boolean allIndividualsEvaluated = false;

   private int probabilities[];
   private int totalIndividualsProgessionSum;

   public Population(PopulationParameters populationParameters, int popNumber)
   {
      this.random = populationParameters.getRandom();
      int numIndividuals = populationParameters.getPopulationSize();
      IndividualToEvaluate individualToEvaluate = populationParameters.getSeedIndividualToEvaluate();
      int numberOfSeedInvidualsToCopy = populationParameters.getNumberOfSeedIndividualsToCopyIntoFirstPopulation();
      System.out.println("numberOfSeedInvidualsToCopy = " + numberOfSeedInvidualsToCopy);
      
      GeneticAlgorithmIndividualToEvaluate geneticAlgorithmIndividualToEvaluate = new GeneticAlgorithmIndividualToEvaluate(individualToEvaluate);
      
      Comparator<GeneticAlgorithmIndividualToEvaluate> comparator = populationParameters.getComparator();
      String name = populationParameters.getName();
      
      
      this.popName = name;
      this.popNumber = popNumber;
      this.comparator = comparator;
      
      generation = new GeneticAlgorithmIndividualToEvaluate[numIndividuals];

      for (int i = 0; i < numIndividuals; i++)
      {
         if (i<numberOfSeedInvidualsToCopy)
         {          
            double mutationRateForCopiedIndividuals = populationParameters.getMutationRateForCopiedIndividuals();
            generation[i] = geneticAlgorithmIndividualToEvaluate.makeCopyOfIndividualAndMutate(random, mutationRateForCopiedIndividuals);
         }
         else
         {
            generation[i] = geneticAlgorithmIndividualToEvaluate.makeRandomIndividual(random);
         }
         generation[i].setName(this.getName() + "_" + this.getPopulationNumber() + "_" + i);
      }

      computeProbabilities();
      
   }
//   public Population(int numIndividuals, GeneticAlgorithmIndividualToEvaluate individualToEvaluate, Comparator<GeneticAlgorithmIndividualToEvaluate> comparator, String name, int popNumber)
//   {
//      this.popName = name;
//      this.popNumber = popNumber;
//      this.comparator = comparator;
//      
//      generation = new GeneticAlgorithmIndividualToEvaluate[numIndividuals];
//
//      for (int i = 0; i < numIndividuals; i++)
//      {
//         generation[i] = individualToEvaluate.makeRandomIndividual();
//         generation[i].setName(this.getName() + "_" + this.getPopulationNumber() + "_" + i);
//      }
//
//      computeProbabilities();
//   }

//   private Population(ArrayList<GeneticAlgorithmIndividualToEvaluate> individuals, Comparator<GeneticAlgorithmIndividualToEvaluate> comparator, String name, int popNumber)
//   {
//      this.popName = name;
//      this.popNumber = popNumber;
//      this.comparator = comparator;
//      generation = new GeneticAlgorithmIndividualToEvaluate[individuals.size()];
//
//      for (int i = 0; i < individuals.size(); i++)
//      {
//         GeneticAlgorithmIndividualToEvaluate individual = (GeneticAlgorithmIndividualToEvaluate) individuals.get(i);
//         generation[i] = individual;
//      }
//
//      computeProbabilities();
//
//      // evaluateAllIndividuals();
//   }

   private Population(Random random, int numberOfIndividuals, Comparator<GeneticAlgorithmIndividualToEvaluate> comparator, String name, int popNumber)
   {
      this.random = random;
      
      this.popName = name;
      this.popNumber = popNumber;
      this.comparator = comparator;
      
      generation = new GeneticAlgorithmIndividualToEvaluate[numberOfIndividuals];

      computeProbabilities();
   }



   public boolean allIndividualsEvaluated()
   {
      return allIndividualsEvaluated;
   }

   public String getName()
   {
      return this.popName;
   }

   public int getPopulationNumber()
   {
      return this.popNumber;
   }

   private void evaluateAllIndividuals()
   {
      if (allIndividualsEvaluated)
         return;

      // System.out.println("Starting the Evals");
      for (int i = 0; i < generation.length; i++)
      {
         if (!generation[i].isEvaluationDone())
            generation[i].startEvaluation();
      }

      // System.out.println("Waiting for the Evals to finish");
      for (int i = 0; i < generation.length; i++)
      {
         while (!generation[i].isEvaluationDone())
         {
            try
            {
               Thread.sleep(1000);
            }
            catch (InterruptedException e)
            {
            }
         }
         
         
         notifyEvaluatedIndividualListeners(generation[i].getIndividualToEvaluate());  
      }

      allIndividualsEvaluated = true;
   }
   

   public int getNumberOfIndividuals()
   {
      return generation.length;
   }
   
   public void evaluateAndSortByFitness()
   {
      evaluateAllIndividuals();
      Arrays.sort(generation, this.comparator);
   }

   public GeneticAlgorithmIndividualToEvaluate[] getAllIndividuals()
   {
      return (generation);
   }

   public GeneticAlgorithmIndividualToEvaluate getIndividual(int index)
   {
      return generation[index];
   }

   public Population breed(double crossoverRate, double mutationRate)
   {
      Population retPop = new Population(this.random, generation.length, this.comparator, this.popName, this.popNumber + 1);

      GeneticAlgorithmIndividualToEvaluate parent1, parent2;
      GeneticAlgorithmIndividualToEvaluate[] children = new GeneticAlgorithmIndividualToEvaluate[2];

      evaluateAndSortByFitness();

      for (int i = 0; i < (generation.length / 2); i++)
      {
         // System.out.println("i: " + i);

         parent1 = selectRandomParent();
         parent2 = selectRandomParent();

         if (crossoverRate > random.nextDouble())
         {
            children = GeneticAlgorithmIndividualToEvaluate.mate(random, parent1, parent2, mutationRate);

            children[0].setName(this.popName + "_" + (this.popNumber + 1) + "_" + i * 2);
            children[1].setName(this.popName + "_" + (this.popNumber + 1) + "_" + (i * 2 + 1));

            retPop.generation[2 * i] = children[0];
            retPop.generation[(2 * i) + 1] = children[1];
         }
         else
         {
            retPop.generation[2 * i] = parent1;
            retPop.generation[(2 * i) + 1] = parent2;
         }
      }

      return retPop;
   }

   private void computeProbabilities()
   {
      probabilities = new int[generation.length];

      // The fittest individual has an         n/(1+2+3+...+n) chance of being selected.
      // The next fittest has an            (n-1)/(1+2+3+...+n) chance.
      // ...                               ...
      // The least fit individual has a         1/(1+2+3+...+n) chance.

      totalIndividualsProgessionSum = 0;

      for (int i = 1; i <= generation.length; i++)
      {
         totalIndividualsProgessionSum = totalIndividualsProgessionSum + i;
         probabilities[i - 1] = totalIndividualsProgessionSum;
      }

      // System.out.println("totalIndividuals: " + totalIndividuals);

      // System.out.println("probabilities: ");
      // for(int i=0; i<probabilities.length; i++)
      // System.out.println(probabilities[i]);

   }

   private GeneticAlgorithmIndividualToEvaluate selectRandomParent()
   {
      int randomIndex = (int) (random.nextDouble() * (totalIndividualsProgessionSum)) + 1;

      // System.out.println("ran: " + ran); System.out.flush();

      // If randomIndex is equal to or less than the value, select it.
      int parentIndex = Arrays.binarySearch(probabilities, randomIndex);

      // System.out.println("parentIndex: " + parentIndex); System.out.flush();
      if (parentIndex < 0)
         parentIndex = -parentIndex - 1;    // This needs to be done due to the way binarySearch returns.

      // System.out.println("parentIndex: " + parentIndex); System.out.flush();

      return generation[generation.length - parentIndex - 1];
   }

   public GeneticAlgorithmIndividualToEvaluate getFittestIndividual()
   {
      this.evaluateAndSortByFitness();

      return generation[0];
   }

   public double getMaximumFitness()
   {
      return getFittestIndividual().getFitness();
   }

   public GeneticAlgorithmIndividualToEvaluate getLeastFitIndividual()
   {
      this.evaluateAndSortByFitness();

      return generation[generation.length - 1];
   }

   public double getMinimumFitness()
   {
      return getLeastFitIndividual().getFitness();
   }

   public double getAverageFitness()
   {
      double total = 0.0;

      evaluateAllIndividuals();

      for (int i = 0; i < generation.length; i++)
      {
         total = total + generation[i].getFitness();
      }

      return (total / generation.length);
   }

   public String toString()
   {
      String ret = "Population " + this.getName() + "  Number Individuals: " + generation.length + "\n";

      for (int i = 0; i < generation.length; i++)
      {
         ret = ret + generation[i].toString() + "\n";
      }

      return ret;
   }

   public void save(String filename)
   {
      System.err.println("Save not implemented yet");
//      throw new RuntimeException("Fix me!");
   }
   
//   public void save(String filename)
//   {
//      if (!filename.endsWith(".pop"))
//         filename = filename + ".pop";
//
////    File file = new File(filename);
//
//      try
//      {
//         PrintWriter writer = new PrintWriter(new FileOutputStream(filename));
//
//         writer.println("Population: " + this.getPopulationNumber());
//
//         // writer.println("");
//
//         GeneticAlgorithmIndividualToEvaluate[] individuals = this.getAllIndividuals();
//         for (int indIndex = 0; indIndex < individuals.length; indIndex++)
//         {
//            GeneticAlgorithmIndividualToEvaluate individual = individuals[indIndex];
//
//            // writer.println(individual);
//            individual.writeOut(writer);
//
//            // writer.println("");
//         }
//
//         writer.close();
//      }
//      catch (IOException e)
//      {
//         System.err.println("File IO Error in Population.save()");
//      }
//
//   }

   
   public static Population  load(String filename, String populationName, IndividualToEvaluate individual)
   {
      throw new RuntimeException("Fix me!");
   }
   
//   public static Population load(String filename, String populationName, IndividualToEvaluate individual)
//   {
//      if (!filename.endsWith(".pop"))
//         filename = filename + ".pop";
//
//      // File file = new File(filename);
//
//      try
//      {
//         BufferedReader reader = new BufferedReader(new FileReader(filename));
//
//         String line = reader.readLine();
//         StringTokenizer tokenizer = new StringTokenizer(line);
//         String popString = tokenizer.nextToken();
//
//         if (!popString.equals("Population:"))
//         {
//            System.err.println("File must start with Population: <population number>");
//
//            return null;
//         }
//
//         int populationNumber = Integer.parseInt(tokenizer.nextToken());
//
//         ArrayList<GeneticAlgorithmIndividualToEvaluate> newIndividuals = new ArrayList<GeneticAlgorithmIndividualToEvaluate>();
//         boolean endOfFile = false;
//
//         while (!endOfFile)
//         {
//            GeneticAlgorithmIndividualToEvaluate newIndividual = individual.readIn(reader);
//            if (newIndividual == null)
//               endOfFile = true;
//            else
//            {
//               newIndividuals.add(newIndividual);
//
//               // System.out.println(newIndividual);
//            }
//         }
//
//         reader.close();
//
//         Population newPopulation = new Population(newIndividuals, populationName, populationNumber);
//         Arrays.sort(newPopulation.generation);
//
//         return newPopulation;
//
//      }
//      catch (IOException e)
//      {
//         System.err.println("File IO Error in Population.load()");
//
//         return null;
//      }
//   }
   
   private void notifyEvaluatedIndividualListeners(IndividualToEvaluate individual)
   {
      if (evaluatedIndividualListeners == null) return;
      
      for (EvaluatedIndividualListener listener : evaluatedIndividualListeners)
      {
         listener.evaluatedIndividual(individual);
      }
   }
   
   public void attachEvaluatedIndividualListener(EvaluatedIndividualListener listener)
   {
      if (evaluatedIndividualListeners == null) evaluatedIndividualListeners = new ArrayList<EvaluatedIndividualListener>();
      
      evaluatedIndividualListeners.add(listener);
   }
   
}
