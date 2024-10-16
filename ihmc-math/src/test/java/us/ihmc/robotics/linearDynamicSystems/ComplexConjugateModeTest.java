package us.ihmc.robotics.linearDynamicSystems;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.robotics.dataStructures.ComplexNumber;

import static org.junit.jupiter.api.Assertions.*;

public class ComplexConjugateModeTest
{
   @BeforeEach
   public void setUp()
   {
   }

   @AfterEach
   public void tearDown()
   {
   }

	@Test
   public void testSecondOrderMassSpringDamper()
   {
      double wn = 7.2;
      double zeta = 0.3;
      double P1 = zeta * wn;
      double P2 = Math.sqrt(1.0 - zeta * zeta) * wn;
      @SuppressWarnings("unused")
      double P3 = wn * wn;

      ComplexNumber[] leftEigenvectorV = new ComplexNumber[] {new ComplexNumber(-P2, -P1), new ComplexNumber(0.0, 1.0)};
      ComplexNumber[] rightEigenvectorW = new ComplexNumber[] {new ComplexNumber(-1.0 / (2.0 * P2), 0.0), new ComplexNumber(-P1 / (2.0 * P2), -0.5)};

      ComplexNumber eigenvalue = new ComplexNumber(-P1, P2);

      ComplexConjugateMode complexConjugateMode = new ComplexConjugateMode(eigenvalue, leftEigenvectorV, rightEigenvectorW);
      DynamicSystemsTestHelpers.assertEpsilonEquals(eigenvalue, complexConjugateMode.getEigenvalue(), 1e-7);

      TransferFunctionMatrix transferFunctionMatrix = complexConjugateMode.constructTransferFunctionMatrix();

      @SuppressWarnings("unused") TransferFunction transferFunction00 = transferFunctionMatrix.get(0, 0);
      @SuppressWarnings("unused")
      TransferFunction transferFunction01 = transferFunctionMatrix.get(0, 1);
      TransferFunction transferFunction10 = transferFunctionMatrix.get(1, 0);
      @SuppressWarnings("unused")
      TransferFunction transferFunction11 = transferFunctionMatrix.get(1, 1);
      TransferFunction expectedTransferFunction10 = new TransferFunction(new double[] {1.0}, new double[] {wn * wn,  2.0 * zeta * wn, 1.0 });

      // System.out.println("\nMassSpringDamper:");
      //
      // System.out.print("transferFunction00 = " + transferFunction00);
      // System.out.println("                 transferFunction01 = " + transferFunction01);
      // System.out.print("transferFunction10 = " + transferFunction10);
      // System.out.println("                          transferFunction11 = " + transferFunction11);
      //
      // System.out.println("expectedTransferFunction10 = " + expectedTransferFunction10);

      assertTrue(expectedTransferFunction10.epsilonEquals(transferFunction10, 1e-7));
   }

	@Test
   public void testCircleGenerator()
   {
      ComplexNumber[] leftEigenvectorV = new ComplexNumber[] {new ComplexNumber(1.0, 0.0), new ComplexNumber(0.0, -1.0)};
      ComplexNumber[] rightEigenvectorW = new ComplexNumber[] {new ComplexNumber(0.5, 0.0), new ComplexNumber(0.0, 0.5)};

      ComplexNumber eigenvalue = new ComplexNumber(0.0, 1.0);

      ComplexConjugateMode complexConjugateMode = new ComplexConjugateMode(eigenvalue, leftEigenvectorV, rightEigenvectorW);
      DynamicSystemsTestHelpers.assertEpsilonEquals(eigenvalue, complexConjugateMode.getEigenvalue(), 1e-7);

      TransferFunctionMatrix transferFunctionMatrix = complexConjugateMode.constructTransferFunctionMatrix();

      @SuppressWarnings("unused")
      TransferFunction transferFunction00 = transferFunctionMatrix.get(0, 0);
      TransferFunction transferFunction01 = transferFunctionMatrix.get(0, 1);
      @SuppressWarnings("unused")
      TransferFunction transferFunction10 = transferFunctionMatrix.get(1, 0);
      @SuppressWarnings("unused")
      TransferFunction transferFunction11 = transferFunctionMatrix.get(1, 1);
      TransferFunction expectedTransferFunction01 = new TransferFunction(new double[] {-1.0}, new double[] {1.0, 0.0, 1.0});

      // System.out.println("\nCircleGenerator:");
      //
      // System.out.print("transferFunction00 = " + transferFunction00);
      // System.out.println("        transferFunction01 = " + transferFunction01);
      // System.out.print("transferFunction10 = " + transferFunction10);
      // System.out.println("                   transferFunction11 = " + transferFunction11);
      //
      // System.out.println("expectedTransferFunction01 = " + expectedTransferFunction01);

      assertTrue(expectedTransferFunction01.epsilonEquals(transferFunction01, 1e-7));
   }

}
