package us.ihmc.robotics.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
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
         DMatrixRMaj A = new DMatrixRMaj(1, 1);
         DMatrixRMaj b = new DMatrixRMaj(1, 1);
         A.set(0, EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         b.set(0, EuclidCoreRandomTools.nextDouble(random, 10.0));
         LinearComplementarityProblemSolver solver = new LinearComplementarityProblemSolver();
         DMatrixRMaj f = solver.solve(A, b);

         DMatrixRMaj a = new DMatrixRMaj(1, 1);
         CommonOps_DDRM.mult(A, f, a);
         CommonOps_DDRM.addEquals(a, b);

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
         DMatrixRMaj A = RandomMatrices_DDRM.symmetricPosDef(numberOfContacts, random);
         DMatrixRMaj b = RandomMatrices_DDRM.rectangle(numberOfContacts, 1, -1.0, 1.0, random);

         LinearComplementarityProblemSolver solver = new LinearComplementarityProblemSolver();
         DMatrixRMaj f = solver.solve(A, b);

         DMatrixRMaj a = new DMatrixRMaj(numberOfContacts, 1);

         CommonOps_DDRM.mult(A, f, a);
         CommonOps_DDRM.addEquals(a, b);

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
