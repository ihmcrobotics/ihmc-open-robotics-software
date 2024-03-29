package us.ihmc.parameterEstimation.solver;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;

import static org.junit.jupiter.api.Assertions.*;

public class GarbageFreeDoglegSolverOsborne2Test
{
   public static final double EPSILON = 1.0e-12;

   private GarbageFreeDoglegSolver solver;

   private final DMatrixRMaj x0 = new DMatrixRMaj(new double[] {1.3, 0.65, 0.65, 0.7, 0.6, 3.0, 5.0, 7.0, 2.0, 4.5, 5.5});

   @BeforeEach
   public void setUpSolver()
   {
      double trustRegionRadius = 1.0;
      int maximumIterations = 50;

      solver = new GarbageFreeDoglegSolver(new Osborne2ResidualAndJacobian(), maximumIterations);

      solver.initialize(x0, trustRegionRadius);
   }

   @Test
   public void testSolve()
   {
      solver.setVerbose(true);
      solver.solve();

      assertTrue(solver.isConverged());

      // Because this test function is quite ill-scaled, we omit a check on the solution value (it's too tough to check without using a really loose EPSILON),
      // and instead check only the objective value at the solution, which we know and can test tightly
      double expectedObjective = 0.0200688681467739;
      assertEquals(solver.getObjective(), expectedObjective, EPSILON);
   }

   /**
    * See More, Garbow, and Hillstrom (1981): "Testing Unconstrained Optimization Software".
    * <p>
    * This is the Osborne 2 problem for nonlinear least squares solvers.
    * </p>
    */
   public static class Osborne2ResidualAndJacobian implements GarbageFreeResidualAndJacobian
   {
      DMatrixRMaj data;

      DMatrixRMaj jacobianRow;

      int parameterSize = 11;
      int residualSize = 65;

      Osborne2ResidualAndJacobian()
      {
         jacobianRow = new DMatrixRMaj(1, parameterSize);
         jacobianRow.zero();

         data = new DMatrixRMaj(new double[][] {
               {1, 1.366},
               {2, 1.191},
               {3, 1.112},
               {4, 1.013},
               {5, 0.991},
               {6, 0.885},
               {7, 0.831},
               {8, 0.847},
               {9, 0.786},
               {10, 0.725},
               {11, 0.746},
               {12, 0.679},
               {13, 0.608},
               {14, 0.655},
               {15, 0.616},
               {16, 0.606},
               {17, 0.602},
               {18, 0.626},
               {19, 0.651},
               {20, 0.724},
               {21, 0.649},
               {22, 0.649},
               {23, 0.694},
               {24, 0.644},
               {25, 0.624},
               {26, 0.661},
               {27, 0.612},
               {28, 0.558},
               {29, 0.533},
               {30, 0.495},
               {31, 0.500},
               {32, 0.423},
               {33, 0.395},
               {34, 0.375},
               {35, 0.372},
               {36, 0.391},
               {37, 0.396},
               {38, 0.405},
               {39, 0.428},
               {40, 0.429},
               {41, 0.523},
               {42, 0.562},
               {43, 0.607},
               {44, 0.653},
               {45, 0.672},
               {46, 0.708},
               {47, 0.633},
               {48, 0.668},
               {49, 0.645},
               {50, 0.632},
               {51, 0.591},
               {52, 0.559},
               {53, 0.597},
               {54, 0.625},
               {55, 0.739},
               {56, 0.710},
               {57, 0.729},
               {58, 0.720},
               {59, 0.636},
               {60, 0.581},
               {61, 0.428},
               {62, 0.292},
               {63, 0.162},
               {64, 0.098},
               {65, 0.054}
         });
      }

      @Override
      public void calculateResidual(DMatrixRMaj x, DMatrixRMaj residualToPack)
      {
         double residualEntry;
         for (int i = 0; i < data.numRows; ++i)
         {
            double t = i / 10.0;
            residualEntry = data.get(i, 1) - (x.get(0) * Math.exp(-t * x.get(4)) + x.get(1) * Math.exp(-MathTools.square(t - x.get(8)) * x.get(5)) + x.get(2) * Math.exp(-MathTools.square(t - x.get(9)) * x.get(6)) + x.get(3) * Math.exp(-MathTools.square(t - x.get(10)) * x.get(7)));
            residualToPack.unsafe_set(i, 0, residualEntry);
         }
      }

      @Override
      public void calculateJacobian(DMatrixRMaj x, DMatrixRMaj jacobianToPack)
      {
         for (int i = 0; i < data.numRows; ++i)
         {
            double t = i / 10.0;
            jacobianRow.unsafe_set(0, 0, -Math.exp(-t * x.get(4)));
            jacobianRow.unsafe_set(0, 1, -Math.exp(-x.get(5) * MathTools.square(t - x.get(8))));
            jacobianRow.unsafe_set(0, 2, -Math.exp(x.get(6) * -MathTools.square(t - x.get(9))));
            jacobianRow.unsafe_set(0,3, -Math.exp(x.get(7) * -MathTools.square(t - x.get(10))));
            jacobianRow.unsafe_set(0, 4, t * x.get(0) * Math.exp(-t * x.get(4)));
            jacobianRow.unsafe_set(0, 5, x.get(1) * MathTools.square(t - x.get(8)) * Math.exp(x.get(5) * -MathTools.square(t - x.get(8))));
            jacobianRow.unsafe_set(0, 6, x.get(2) * MathTools.square(t - x.get(9)) * Math.exp(x.get(6) * -MathTools.square(t - x.get(9))));
            jacobianRow.unsafe_set(0, 7, x.get(3) * MathTools.square(t - x.get(10)) * Math.exp(x.get(7) * -MathTools.square(t - x.get(10))));
            jacobianRow.unsafe_set(0, 8, -2.0 * x.get(1) * x.get(5) * (t - x.get(8)) * Math.exp(x.get(5) * -MathTools.square(t - x.get(8))));
            jacobianRow.unsafe_set(0, 9, -2.0 * x.get(2) * x.get(6) * (t - x.get(9)) * Math.exp(x.get(6) * -MathTools.square(t - x.get(9))));
            jacobianRow.unsafe_set(0, 10, -2.0 * x.get(3) * x.get(7) * (t - x.get(10)) * Math.exp(x.get(7) * -MathTools.square(t - x.get(10))));
            CommonOps_DDRM.insert(jacobianRow, jacobianToPack, i, 0);
         }
      }

      @Override
      public int getParameterSize()
      {
         return parameterSize;
      }

      @Override
      public int getResidualSize()
      {
         return residualSize;
      }
   }
}
