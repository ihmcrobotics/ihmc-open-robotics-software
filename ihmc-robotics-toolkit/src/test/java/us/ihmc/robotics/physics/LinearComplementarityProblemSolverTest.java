package us.ihmc.robotics.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;

public class LinearComplementarityProblemSolverTest
{
   private static final double EPSILON = 1.0e-12;
   private static final int ITERATIONS = 1000;

   @Test
   public void testSingleton()
   {
      Random random = new Random(32457);

      for (int i = 0; i < ITERATIONS; i++)
      {
         String prefix = "Iteration: " + i + ": ";
         DenseMatrix64F A = new DenseMatrix64F(1, 1);
         DenseMatrix64F b = new DenseMatrix64F(1, 1);
         A.set(0, EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         b.set(0, EuclidCoreRandomTools.nextDouble(random, 10.0));
         LinearComplementarityProblemSolver solver = new LinearComplementarityProblemSolver();
         DenseMatrix64F f = solver.solve(A, b);

         DenseMatrix64F a = new DenseMatrix64F(1, 1);
         CommonOps.mult(A, f, a);
         CommonOps.addEquals(a, b);

         double a_i = a.get(0);
         double f_i = f.get(0);

         assertTrue(a_i > -EPSILON, prefix + "the contact acceleration is negative: " + a_i);
         assertTrue(f_i > -EPSILON, prefix + "the contact force is negative: " + f_i);
         assertEquals(0.0, f_i * a_i, EPSILON, prefix + "the contact force x acceleration is non-zero.");
      }
   }

   @Test
   public void testRandomProblems()
   {
      Random random = new Random(43536);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfContacts = random.nextInt(50) + 1;
         DenseMatrix64F A = RandomMatrices.createSymmPosDef(numberOfContacts, random);
         DenseMatrix64F b = RandomMatrices.createRandom(numberOfContacts, 1, -1.0, 1.0, random);

         LinearComplementarityProblemSolver solver = new LinearComplementarityProblemSolver();
         DenseMatrix64F f = solver.solve(A, b);

         DenseMatrix64F a = new DenseMatrix64F(numberOfContacts, 1);

         CommonOps.mult(A, f, a);
         CommonOps.addEquals(a, b);

         for (int contactIndex = 0; contactIndex < numberOfContacts; contactIndex++)
         {
            double a_i = a.get(contactIndex);
            double f_i = f.get(contactIndex);

            String prefix = "Iteration: " + i + ": ";
            assertTrue(a_i > -EPSILON, prefix + contactIndex + "th contact acceleration is negative: " + a_i);
            assertTrue(f_i > -EPSILON, prefix + contactIndex + "th contact force is negative: " + f_i);
            assertEquals(0.0, f_i * a_i, EPSILON, prefix + contactIndex + "th contact force x acceleration is non-zero.");
         }
      }
   }

}
