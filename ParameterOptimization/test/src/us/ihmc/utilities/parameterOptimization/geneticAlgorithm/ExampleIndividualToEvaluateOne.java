package us.ihmc.utilities.parameterOptimization.geneticAlgorithm;

import us.ihmc.utilities.parameterOptimization.IndividualToEvaluate;
import us.ihmc.utilities.parameterOptimization.IntegerParameterToOptimize;
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

public class ExampleIndividualToEvaluateOne extends IndividualToEvaluate
{
   private final ListOfParametersToOptimize listOfParametersToOptimize;
   
   private static final String stringToMatch = "Jim Warrenfeltz";
      
   private char[] testString = new char[stringToMatch.length()];
   
   
   public ExampleIndividualToEvaluateOne()
   {
      listOfParametersToOptimize = new ListOfParametersToOptimize();

      for (int i=0; i<stringToMatch.length(); i++)
      {
         IntegerParameterToOptimize integerParameterToOptimize = new IntegerParameterToOptimize("letter_"+i, 32, 125, listOfParametersToOptimize);
         integerParameterToOptimize.setCurrentValue(67);
      }
   }
   

   public ListOfParametersToOptimize getStructuralParametersToOptimize()
   {
      return null;
   }

   public ListOfParametersToOptimize getControlParametersToOptimize()
   {
      return listOfParametersToOptimize;
   }
   
   
   public IndividualToEvaluate createNewIndividual()
   {
      return new ExampleIndividualToEvaluateOne();
   }

   public void startEvaluation()
   {
   }

   public boolean isEvaluationDone()
   {
      return true;
   }

   public synchronized double computeFitness()
   {
      for (int i = 0; i < stringToMatch.length(); i++)
      {
         IntegerParameterToOptimize parameter = (IntegerParameterToOptimize) listOfParametersToOptimize.get(i);
         int letterIndex = parameter.getCurrentValue();

         testString[i] = (char) letterIndex;
      }

      int fitness = 0;

      for (int i = 0; i < stringToMatch.length(); i++)
      {
         if (testString[i] == stringToMatch.charAt(i))
            fitness = fitness + 1;
      }

      return fitness;
   }

//   public void writeOut(PrintWriter printWriter)
//   {
//      printWriter.print("testString: ");
//
//      for (int i = 0; i < testString.length; i++)
//      {
//         printWriter.print(testString[i]);
//      }
//
//      printWriter.println(" fitness: " + getFitness() + "  " + this.getGenotype());
//   }

//   public GeneticAlgorithmIndividualToEvaluate readIn(BufferedReader bufferedReader) throws java.io.IOException
//   {
//      IndividualOne ret = new IndividualOne();
//
//      String line = bufferedReader.readLine();
//      if (line == null)
//         return null;
//
//
//      // line.indexOf("testString:") + 11
//
//      ret.testString = (line.substring(12, 12 + stringToMatch.length())).toCharArray();
//
//      // System.out.println(ret.testString);
//
//      StringTokenizer tokenizer = new StringTokenizer(line, " ", false);
//      while (tokenizer.hasMoreTokens())
//      {
//         String token = tokenizer.nextToken();
//         if (token.equals("fitness:"))
//            ret.setFitness(Double.parseDouble(tokenizer.nextToken()));
//      }
//
//      double[] phenotype = new double[stringToMatch.length()];
//
//      for (int i = 0; i < stringToMatch.length(); i++)
//      {
//         int code = (int) ret.testString[i];
//         double code2 = code - 32.0;
//
//         phenotype[i] = code2 / (125.0 - 32.0);
//
//         // System.out.println(ret.testString[i] + " " + code + " " + code2 + " " + phenotype[i]);
//      }
//
//
//      Genotype genes = ret.getGenotype();
//      genes.setDoublePhenotype(phenotype);
//
//      return ret;
//   }

   public String toString()
   {
      String ret = new String(testString);
      ret = ret + "   :" + this.getFitness();    // + " " + this.getGenotype();

      return ret;
   }



}
