package us.ihmc.utilities.parameterOptimization.geneticAlgorithm;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class GenotypeTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGenotype()
   {
      Random random = new Random(1776L);
      
      int[] bitsPerGene = new int[] { 4, 6, 8 };
      int nBits = 0;

      for (int i = 0; i < bitsPerGene.length; i++)
      {
         nBits = nBits + bitsPerGene[i];
      }

      Genotype test = new Genotype(bitsPerGene);
      Genotype test2 = new Genotype(bitsPerGene);

      int temp1[] = new int[nBits];
      int temp2[] = new int[nBits];

      for (int i = 0; i < nBits; i++)
      {
         temp1[i] = 0;
         temp2[i] = 1;
      }

      test.setBits(temp1);
      test2.setBits(temp2);

      double[] doublePhenotype = test.getDoublePhenotype();
      assertEquals(3, doublePhenotype.length);
      assertEquals(doublePhenotype[0], 0.0, 0.01);
      assertEquals(doublePhenotype[1], 0.0, 0.01);
      assertEquals(doublePhenotype[2], 0.0, 0.01);
      
      double[] doublePhenotype2 = test2.getDoublePhenotype();
      assertEquals(3, doublePhenotype2.length);
      assertEquals(doublePhenotype2[0], 0.9375, 1e-5);
      assertEquals(doublePhenotype2[1], 0.984375, 1e-5);
      assertEquals(doublePhenotype2[2], 0.99609375, 1e-5);
      
      assertEquals(3, test.getNumberOfGenes());
      assertEquals(3, test2.getNumberOfGenes());
      
      assertEquals(18, test.getTotalNumberOfBits());
      assertEquals(18, test2.getTotalNumberOfBits());
      
      // Make some children and make sure they are similar to their parents:
      Genotype[] children = test.crossover(random, test2, 0.002);

      assertEquals(3, children[0].getNumberOfGenes());
      assertEquals(3, children[1].getNumberOfGenes());
      
      assertEquals(18, children[0].getTotalNumberOfBits());
      assertEquals(18, children[1].getTotalNumberOfBits());

      // Extract the info, make a new Genotype, and make sure it has the same phenotype:
      int[] bits = children[0].getBits();
      Genotype newGenotype = new Genotype(children[0].getBitsPerGene());
      newGenotype.setBits(bits);
      assertDoubleArraysEqual(children[0].getDoublePhenotype(), newGenotype.getDoublePhenotype(), 1e-7);

      int[] bits2 = children[1].getBits();
      Genotype newGenotype2 = new Genotype(children[1].getBitsPerGene());
      newGenotype2.setBits(bits2);
      assertDoubleArraysEqual(children[1].getDoublePhenotype(), newGenotype2.getDoublePhenotype(), 1e-7);
   }
   
   
   private void assertDoubleArraysEqual(double[] array1, double[] array2, double epsilon)
   {
      assertEquals(array1.length, array2.length);
      
      for (int i=0; i<array1.length; i++)
      {
         assertEquals(array1[i], array2[i], epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000) 
   public void testGenotypeAtExactlyOneAndZero()
   {
      Genotype genotype = new Genotype(new int[] { 8, 8 });

      genotype.setDoublePhenotype(new double[]{0.0, 1.0});
      int[] bits = genotype.getBits();
      
      for (int i=0; i<16; i++)
      {
         if (i<8) assertEquals(0, bits[i]);
         else assertEquals(1, bits[i]);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000) 
   public void testGenotypeNearOneAndZero()
   {
      Genotype genotype = new Genotype(new int[] { 8, 8 });

      genotype.setDoublePhenotype(new double[]{0.001, 0.999});
      int[] bits = genotype.getBits();
      
      for (int i=0; i<16; i++)
      {
         if (i<8) assertEquals(0, bits[i]);
         else assertEquals(1, bits[i]);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPhenotypeTwo()
   {
      Genotype genotype = new Genotype(new int[] { 8, 8 });

      double[] phenotypeIn = new double[2], phenotypeOut;

      int numberOfTests = 100;
      
      for (int i = 0; i < numberOfTests; i++)
      {
         phenotypeIn[0] = Math.random();
         phenotypeIn[1] = Math.random();

         genotype.setDoublePhenotype(phenotypeIn);
         phenotypeOut = genotype.getDoublePhenotype();
         
         double epsilon = 2.0 * 1.0/256.0;
         
         assertEquals(phenotypeIn[0], phenotypeOut[0], epsilon);
         assertEquals(phenotypeIn[1], phenotypeOut[1], epsilon);
      } 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000) 
   public void testPhenotypeReconstruction()
   {
      Random random = new Random(1984L);
      
      int numberOfTests = 1000;

      for (int i=0; i<numberOfTests; i++)
      {
         Genotype genotype = new Genotype(new int[]{16, 16});

         genotype.setRandomGenes(random);

         double[] phenotype = genotype.getDoublePhenotype();

         Genotype genotype2 = new Genotype(new int[]{16, 16});
         genotype2.setDoublePhenotype(phenotype);

         double[] phenotype2 = genotype.getDoublePhenotype();

         assertDoubleArraysEqual(phenotype, phenotype2, 1e-7);

      }
   }

}
