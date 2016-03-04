package us.ihmc.utilities.parameterOptimization.geneticAlgorithm;

import java.util.Random;

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

public class Genotype
{
   // Each bit is represented as an integer that is either 0 or 1.
   // These are then convertered into an array of integers. It is up to the Individuals to interpret the integers.
   // getDoublePhenotype() is a helper method that will convert these to phenotypes in the range of 0.0 to 1.0.

   private int DNA[];
   private int bitsPerGene[];

   public Genotype(Genotype genotypeToCopy)
   {
      this(genotypeToCopy.bitsPerGene);
      
      for (int i=0; i<DNA.length; i++)
      {
         this.DNA[i] = genotypeToCopy.DNA[i]; 
      }
   }
   
   public Genotype(int[] bitsPerGene)
   {
      int totalDNALength = 0;
      this.bitsPerGene = new int[bitsPerGene.length];

      for (int i = 0; i < bitsPerGene.length; i++)
      {
         int bits = bitsPerGene[i];
         if ((bits < 1) || (bits > 24))
         {
            throw new RuntimeException("Bits per gene must be > 0 and <=24!");
         }

         this.bitsPerGene[i] = bitsPerGene[i];
         totalDNALength = totalDNALength + bitsPerGene[i];
      }

      DNA = new int[totalDNALength];
   }
   
   public int[] getBitsPerGene()
   {
      return bitsPerGene;
   }

   public void setRandomGenes(Random random)
   {
      int totalDNALength = 0;

      for (int i = 0; i < bitsPerGene.length; i++)
      {
         totalDNALength = totalDNALength + bitsPerGene[i];
      }

      for (int i = 0; i < totalDNALength; i++)
      {
         double x = random.nextDouble();

         if (x >= 0.5)
            DNA[i] = 1;
         else
            DNA[i] = 0;
      }
   }

   public double[] getDoublePhenotype()
   {
      // Will return the phenotypes from 0.0 to 1.0

      double[] ret = new double[bitsPerGene.length];
      int DNAIndex = 0;

      for (int i = 0; i < bitsPerGene.length; i++)
      {
//       double code = 0.0;
         long longCode = 0;
         long longBit = 1;

         for (int j = 0; j < bitsPerGene[i]; j++)
         {
            int bit = DNA[DNAIndex + j];

//          code = code + (bit * ( (int) Math.pow(2.0, j)));
            if (bit == 1)
            {
               longCode = longCode + longBit;
            }

            longBit = longBit << 1;
         }

//       code = code / Math.pow(2.0, bitsPerGene[i]);
         double longCodeToDouble = ((double) longCode) / ((double) longBit);

         ret[i] = longCodeToDouble;

//       ret[i] = code;

         DNAIndex = DNAIndex + bitsPerGene[i];
      }

      return ret;
   }

   public void setDoublePhenotype(double[] phenotype)
   {
      // Will set the genes, based on phenotypes from 0.0 to 1.0-epsilon
      int DNAIndex = 0;

      for (int i = 0; i < phenotype.length; i++)
      {
         // Convert double from 0.0 to 1.0 to integer of bitsPerGene[i] bits
         double phen = phenotype[i];
         if (phen < 0.0)
            phen = 0.0;
         if (phen > 1.0 - 1e-17)
            phen = 1.0 - 1e-17;    // If we don't do this, then 1.0 might go to 0.0?

//       if (phen > 1.0) phen = 1.0; 

         long foo = (1 << bitsPerGene[i]) - 1;
         int code = ((int) Math.round(phen * ((double) foo)));

//       int code = ((int) (Math.floor(phen * (Math.pow(2.0, bitsPerGene[i]) - 1))));

         for (int j = 0; j < bitsPerGene[i]; j++)
         {
            DNA[DNAIndex + j] = code & 0x01;
            code = code >> 1;
         }

         DNAIndex = DNAIndex + bitsPerGene[i];
      }
   }



   public String toString()
   {
      String ret = "";

      for (int i = 0; i < DNA.length; i++)
      {
         if (DNA[i] == 1)
            ret = ret + "1";
         else
            ret = ret + "0";
      }

      return ret;
   }

   public int[] getBits()
   {
      return DNA;
   }

   public int getTotalNumberOfBits()
   {
      return (DNA.length);
   }

   public int getNumberOfGenes()
   {
      return bitsPerGene.length;
   }

   public void setBits(int[] bitsToSet)
   {
      for (int i = 0; i < DNA.length; i++)
      {
         DNA[i] = bitsToSet[i];
      }
   }

   public void mutate(Random random, double rate)
   {
      for (int i = 0; i < DNA.length; i++)
      {
         double x = random.nextDouble();    
         if (x < rate)
         {
            if (DNA[i] == 1)
               DNA[i] = 0;
            else
               DNA[i] = 1;
         }
      }
   }

   public Genotype[] crossover(Random random, Genotype g, double mutrate)
   {
      Genotype ret[] = new Genotype[2];
      ret[0] = new Genotype(bitsPerGene);
      ret[1] = new Genotype(bitsPerGene);

      int child1DNA[] = new int[DNA.length];
      int child2DNA[] = new int[DNA.length];

      // System.out.println("testInd:      "+ DNA.length);
      // System.out.println("testInd2:      " + g);

      int[] g1DNA = getBits();
      int[] g2DNA = g.getBits();

      int numBits = getTotalNumberOfBits();

      int splitBit = (int) (random.nextDouble() * (numBits - 1));
      int splitBit2 = (int) (random.nextDouble() * (numBits - 1));

      boolean outsideSplit = false;
      if (splitBit2 < splitBit)
      {
         int temp = splitBit;
         splitBit = splitBit2;
         splitBit2 = temp;

         outsideSplit = true;
      }

      for (int i = 0; i <= splitBit; i++)
      {
         child1DNA[i] = g1DNA[i];
         child2DNA[i] = g2DNA[i];
      }

      for (int i = splitBit + 1; i <= splitBit2; i++)
      {
         child1DNA[i] = g2DNA[i];
         child2DNA[i] = g1DNA[i];
      }

      for (int i = splitBit2 + 1; i < numBits; i++)
      {
         child1DNA[i] = g1DNA[i];
         child2DNA[i] = g2DNA[i];
      }

      if (outsideSplit)
      {
         ret[1].DNA = child1DNA;
         ret[0].DNA = child2DNA;
      }
      else
      {
         ret[0].DNA = child1DNA;
         ret[1].DNA = child2DNA;
      }

      ret[0].mutate(random, mutrate);
      ret[1].mutate(random, mutrate);

      return ret;
   }

}
