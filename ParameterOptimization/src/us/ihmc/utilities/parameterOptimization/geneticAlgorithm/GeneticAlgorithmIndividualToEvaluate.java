package us.ihmc.utilities.parameterOptimization.geneticAlgorithm;

import us.ihmc.utilities.parameterOptimization.IndividualToEvaluate;
import us.ihmc.utilities.parameterOptimization.ListOfParametersToOptimize;

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

public class GeneticAlgorithmIndividualToEvaluate
{
   private final IndividualToEvaluate individualToEvaluate;
   
   private Genotype genotype;

   public GeneticAlgorithmIndividualToEvaluate(IndividualToEvaluate individualToEvaluate)
   {
      this.individualToEvaluate = individualToEvaluate;
   }
   
   public IndividualToEvaluate getIndividualToEvaluate()
   {
      return individualToEvaluate;
   }
   
   
   public String getName()
   {
      return individualToEvaluate.getName();
   }
   
   public void setName(String name)
   {
      individualToEvaluate.setName(name);
   }
   
   public boolean isEvaluationDone()
   {
      return individualToEvaluate.isEvaluationDone();
   }
   
   public void startEvaluation()
   {
      individualToEvaluate.startEvaluation();
   }
   
   public double getFitness()
   {
      return individualToEvaluate.getFitness();
   }
   
   
   public String toString()
   {
      return individualToEvaluate.toString();
   }

   public Genotype getGenotype()
   {
      return genotype;
   }

   public void setGenotype(Genotype genes)
   {
      this.genotype = genes;
      double[] phenotype = genes.getDoublePhenotype();
      
      ListOfParametersToOptimize listOfParametersToOptimize = this.individualToEvaluate.getAllParametersToOptimize();
      listOfParametersToOptimize.setCurrentValuesGivenZeroToOnes(phenotype);
   }

   public static GeneticAlgorithmIndividualToEvaluate[] mate(GeneticAlgorithmIndividualToEvaluate parent1, GeneticAlgorithmIndividualToEvaluate parent2, double mutate_rate)
   {
      GeneticAlgorithmIndividualToEvaluate children[] = new GeneticAlgorithmIndividualToEvaluate[2];

      children[0] = parent1.makeNewIndividual();
      children[1] = parent1.makeNewIndividual();

      Genotype genotypeForTwoChildren[] = new Genotype[2];
      genotypeForTwoChildren = parent1.getGenotype().crossover(parent2.getGenotype(), mutate_rate);

      children[0].setGenotype(genotypeForTwoChildren[0]);
      children[1].setGenotype(genotypeForTwoChildren[1]);

      return children;
   } 
   

   public static GeneticAlgorithmIndividualToEvaluate makeRandomIndividual(IndividualToEvaluate individualToCopyFrom)
   {
      IndividualToEvaluate individualToEvaluate = individualToCopyFrom.createNewIndividual();
      
      ListOfParametersToOptimize listOfParametersToOptimize = individualToEvaluate.getAllParametersToOptimize();
      
      Genotype genotype = new Genotype(listOfParametersToOptimize.getBitsOfResolution());
      genotype.setRandomGenes();
      double[] doublePhenotype = genotype.getDoublePhenotype();
      listOfParametersToOptimize.setCurrentValuesGivenZeroToOnes(doublePhenotype);
      
      GeneticAlgorithmIndividualToEvaluate ret = new GeneticAlgorithmIndividualToEvaluate(individualToEvaluate);
      ret.setGenotype(genotype);
      
      return ret;
   }
   
   public static GeneticAlgorithmIndividualToEvaluate makeNewIndividual(IndividualToEvaluate individualToCopyFrom)
   {
      IndividualToEvaluate individualToEvaluate = individualToCopyFrom.createNewIndividual();
     
      GeneticAlgorithmIndividualToEvaluate ret = new GeneticAlgorithmIndividualToEvaluate(individualToEvaluate);
      
      return ret;
   }
   
   
   public GeneticAlgorithmIndividualToEvaluate makeRandomIndividual()
   {
      return makeRandomIndividual(this.individualToEvaluate);
   }
   
   public GeneticAlgorithmIndividualToEvaluate makeNewIndividual()
   {
      return makeRandomIndividual(this.individualToEvaluate);
   }

}
