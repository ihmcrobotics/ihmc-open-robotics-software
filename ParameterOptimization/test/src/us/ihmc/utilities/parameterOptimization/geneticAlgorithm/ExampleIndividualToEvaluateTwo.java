package us.ihmc.utilities.parameterOptimization.geneticAlgorithm;

import us.ihmc.utilities.parameterOptimization.DoubleParameterToOptimize;
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

public class ExampleIndividualToEvaluateTwo extends IndividualToEvaluate
{
   private final ListOfParametersToOptimize listOfParametersToOptimize;
   private final DoubleParameterToOptimize xToOptimize, yToOptimize, zToOptimize;
   
   public static int[] getBitsPerGene()
   {
      int[] bitsPerGene = new int[3];

      bitsPerGene[0] = 8;
      bitsPerGene[1] = 16;
      bitsPerGene[2] = 8;

      return bitsPerGene;
   }

   
   public ExampleIndividualToEvaluateTwo()
   {
      listOfParametersToOptimize = new ListOfParametersToOptimize();

      xToOptimize = new DoubleParameterToOptimize("x", 0.0, 2.0, listOfParametersToOptimize);
      yToOptimize = new DoubleParameterToOptimize("y", 0.0, 2.0, listOfParametersToOptimize);
      zToOptimize = new DoubleParameterToOptimize("z", 0.0, 2.0, listOfParametersToOptimize);
   }
   

   public IndividualToEvaluate createNewIndividual()
   {
      return new ExampleIndividualToEvaluateTwo();
   }

   
   public void startEvaluation()
   {
   }

   public boolean isEvaluationDone()
   {
      return true;
   }

   public double computeFitness()
   {
      double x = xToOptimize.getCurrentValue();
      double y = yToOptimize.getCurrentValue();
      double z = zToOptimize.getCurrentValue();
      
      return (Math.abs(Math.cos(x * 2.0 * Math.PI * 10.0)) * Math.abs(Math.sin(y * 2.0 * Math.PI * 10.0)) * (1.0 - (z - 0.5) * (z - 0.5)));

      // return (x + y + z);
      // return Math.abs(Math.cos((x+2.0*y)*2.0*Math.PI*10.0)) * Math.abs(1.0/(Math.abs(z-1.2) + 0.1));
   }

//   public void writeOut(PrintWriter printWriter)
//   {
//      printWriter.println("x: " + x + " y: " + y + " z: " + z + "  fitness: " + getFitness());
//   }

//   public IndividualToEvaluate readIn(BufferedReader bufferedReader) throws java.io.IOException
//   {
//      ListOfParametersToOptimize listOfParametersToOptimize = constructListOfParametersToOptimize();
//      
//      DoubleParameterToOptimize xParameterToOptimize = (DoubleParameterToOptimize) listOfParametersToOptimize.get(0);
//      DoubleParameterToOptimize yParameterToOptimize = (DoubleParameterToOptimize) listOfParametersToOptimize.get(1);
//      DoubleParameterToOptimize zParameterToOptimize = (DoubleParameterToOptimize) listOfParametersToOptimize.get(2);
//      
//      String line = bufferedReader.readLine();
//      if (line == null)
//         return null;
//
//      StringTokenizer tokenizer = new StringTokenizer(line);
//
//      while (tokenizer.hasMoreTokens())
//      {
//         String token = tokenizer.nextToken();
//         String nextToken = tokenizer.nextToken();
//
//         // System.out.println(token);
//         // System.out.println(nextToken);
//         System.out.flush();
//
//         if (token.equals("x:"))
//            xParameterToOptimize.setCurrentValue(Double.parseDouble(nextToken));
//         else if (token.equals("y:"))
//            yParameterToOptimize.setCurrentValue(Double.parseDouble(nextToken));
//         else if (token.equals("z:"))
//            zParameterToOptimize.setCurrentValue(Double.parseDouble(nextToken));
////         else if (token.equals("fitness:"))
////            ret.setFitness(Double.parseDouble(nextToken));         
//      }
//      
//      IndividualTwo ret = new IndividualTwo(listOfParametersToOptimize);
//
//
////      double[] phenotype = new double[] {ret.x, ret.y, ret.z};
////
////      Genotype genes = ret.getGenotype();
////      genes.setDoublePhenotype(phenotype);
//
//      return ret;
//   }


   public ListOfParametersToOptimize getControlParametersToOptimize()
   {
      return listOfParametersToOptimize;
   }
   
   public ListOfParametersToOptimize getStructuralParametersToOptimize()
   {
      return null;
   }


   public String toString()
   {
      String ret = "x: " + xToOptimize.getCurrentValue() + "\ny: " + yToOptimize.getCurrentValue() + "\nz: " + zToOptimize.getCurrentValue();
      ret = ret + "\nfitness: " + this.getFitness();

//      ret = ret + "\n" + this.getGenotype().toString();

      return ret;
   }
}
