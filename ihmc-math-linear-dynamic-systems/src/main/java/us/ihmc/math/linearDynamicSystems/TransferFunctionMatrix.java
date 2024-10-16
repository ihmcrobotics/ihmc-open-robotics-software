package us.ihmc.math.linearDynamicSystems;

import Jama.Matrix;
import org.ejml.simple.SimpleMatrix;
import us.ihmc.math.ComplexConjugateMode;
import us.ihmc.math.ComplexMatrix;
import us.ihmc.math.ComplexNumber;
import us.ihmc.math.SingleRealMode;
import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;

/**
 * Represents a Multi-Input, Multi-Output system of transfer functions, formulated as
 * <p>
 *    H = [H<sub>0,0</sub> H<sub>0,1</sub>; H<sub>1,0</sub>, H<sub>1, 1</sub>]
 * </p>
 * <p>
 *    for example
 * </p>
 */
public class TransferFunctionMatrix
{
   private final TransferFunction[][] transferFunctions;

   public TransferFunctionMatrix(TransferFunction[][] transferFunctions)
   {
      int rows = transferFunctions.length;
      int columns = transferFunctions[0].length;

      this.transferFunctions = new TransferFunction[rows][columns];

      for (int i = 0; i < rows; i++)
      {
         for (int j = 0; j < columns; j++)
         {
            this.transferFunctions[i][j] = transferFunctions[i][j];
         }
      }
   }

   public TransferFunctionMatrix(PolynomialReadOnly[][] numerators, PolynomialReadOnly denominator)
   {
      int rows = numerators.length;
      int columns = numerators[0].length;

      this.transferFunctions = new TransferFunction[rows][columns];

      for (int i = 0; i < rows; i++)
      {
         for (int j = 0; j < columns; j++)
         {
            this.transferFunctions[i][j] = new TransferFunction(numerators[i][j], denominator);
         }
      }
   }

   public static TransferFunctionMatrix constructTransferFunctionMatrix(SingleRealMode singleRealMode)
   {
      Matrix vwT = singleRealMode.getLeftEigenvectorVCopy().times(singleRealMode.getRightEigenvectorWCopy());

      double[] denominator = new double[] {-singleRealMode.getEigenvalue(), 1.0};

      int numRows = vwT.getRowDimension();
      int numColumns = vwT.getColumnDimension();
      TransferFunction[][] transferFunctions = new TransferFunction[numRows][numColumns];

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            double[] numerator = new double[] {vwT.get(i, j)};
            TransferFunction transferFunction = new TransferFunction(numerator, denominator);
            transferFunctions[i][j] = transferFunction;
         }
      }

      return new TransferFunctionMatrix(transferFunctions);
   }

   public static TransferFunctionMatrix constructTransferFunctionMatrix(ComplexConjugateMode complexConjugateMode)
   {
      int order = complexConjugateMode.getLeftEigenvectorVCopy().length;
      TransferFunction[][] transferFunctions = new TransferFunction[order][order];

      Polynomial denominatorPolynomial = new Polynomial(complexConjugateMode.getEigenvalue().magnitudeSquared(), -2.0 * complexConjugateMode.getEigenvalue().real(), 1.0);

      for (int i = 0; i < order; i++)
      {
         for (int j = 0; j < order; j++)
         {
            ComplexNumber Rij = complexConjugateMode.getLeftEigenvectorVCopy()[i].times(complexConjugateMode.getRightEigenvectorWCopy()[j]);

            double[] numeratorCoefficients = new double[] {-2.0 * complexConjugateMode.getEigenvalue().real() * Rij.real() - 2.0 * complexConjugateMode.getEigenvalue().imag() * Rij.imag(), 2.0 * Rij.real() };
            Polynomial numeratorPolynomial = new Polynomial(numeratorCoefficients);

            transferFunctions[i][j] = new TransferFunction(numeratorPolynomial, denominatorPolynomial);
         }
      }

      return new TransferFunctionMatrix(transferFunctions);
   }

   public TransferFunction get(int row, int column)
   {
      return transferFunctions[row][column];
   }

   public int getRows()
   {
      return transferFunctions.length;
   }

   public int getColumns()
   {
      return transferFunctions[0].length;
   }

   public TransferFunctionMatrix plus(TransferFunctionMatrix transferFunctionMatrix)
   {
      int numRows = transferFunctionMatrix.getRows();
      int numColumns = transferFunctionMatrix.getRows();

      if ((numRows != this.getRows()) || (numColumns != this.getColumns()))
      {
         throw new RuntimeException("TransferFunctionMatrix dimensions do not agree!");
      }

      TransferFunction[][] newTransferFunctions = new TransferFunction[numRows][numColumns];

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            newTransferFunctions[i][j] = this.get(i, j).plus(transferFunctionMatrix.get(i, j));
         }
      }

      return new TransferFunctionMatrix(newTransferFunctions);
   }


   public TransferFunctionMatrix preMultiply(SimpleMatrix matrix)
   {
      int numRows = matrix.numRows();
      int numColumns = this.getColumns();
      int innerDimension = this.getRows();

      if (innerDimension != matrix.numCols())
      {
         throw new RuntimeException("TransferFunctionMatrix inner dimensions do not agree!");
      }

      TransferFunction[][] newTransferFunctions = new TransferFunction[numRows][numColumns];

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            newTransferFunctions[i][j] = new TransferFunction(new double[] {0.0}, new double[] {1.0});

            for (int k = 0; k < innerDimension; k++)
            {
               newTransferFunctions[i][j] = newTransferFunctions[i][j].plus(this.get(k, j).times(matrix.get(i, k)));
            }
         }
      }

      return new TransferFunctionMatrix(newTransferFunctions);
   }


   public TransferFunctionMatrix times(SimpleMatrix matrix)
   {
      int numRows = this.getRows();
      int numColumns = matrix.numCols();
      int innerDimension = this.getColumns();

      if (innerDimension != matrix.numRows())
      {
         throw new RuntimeException("TransferFunctionMatrix inner dimensions do not agree!");
      }

      TransferFunction[][] newTransferFunctions = new TransferFunction[numRows][numColumns];

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            newTransferFunctions[i][j] = new TransferFunction(new double[] {0.0}, new double[] {1.0});

            for (int k = 0; k < innerDimension; k++)
            {
               newTransferFunctions[i][j] = newTransferFunctions[i][j].plus(this.get(i, k).times(matrix.get(k, j)));
            }
         }
      }

      return new TransferFunctionMatrix(newTransferFunctions);
   }

   public TransferFunctionMatrix plus(SimpleMatrix matrix)
   {
      int numRows = matrix.numRows();
      int numColumns = matrix.numCols();

      if ((numRows != this.getRows()) || (numColumns != this.getColumns()))
      {
         throw new RuntimeException("TransferFunctionMatrix dimensions do not agree!");
      }

      TransferFunction[][] newTransferFunctions = new TransferFunction[numRows][numColumns];

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            newTransferFunctions[i][j] = this.get(i, j).plus(matrix.get(i, j));
         }
      }

      return new TransferFunctionMatrix(newTransferFunctions);
   }


   public ComplexMatrix evaluate(ComplexNumber complexNumber)
   {
      int numRows = this.getRows();
      int numColumns = this.getColumns();

      ComplexNumber[][] elements = new ComplexNumber[numRows][numColumns];

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            elements[i][j] = this.transferFunctions[i][j].evaluate(complexNumber);
         }
      }

      return new ComplexMatrix(elements);
   }

   public boolean epsilonEquals(TransferFunctionMatrix transferFunctionMatrix, double epsilon)
   {
      int numRows = this.getRows();
      int numColumns = this.getColumns();

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            if (!transferFunctions[i][j].epsilonEquals(transferFunctionMatrix.transferFunctions[i][j], epsilon))
               return false;
         }
      }

      return true;
   }

   public String toString()
   {
      StringBuilder stringBuilder = new StringBuilder();

      int numRows = this.getRows();
      int numColumns = this.getColumns();

      for (int i = 0; i < numRows; i++)
      {
         for (int j = 0; j < numColumns; j++)
         {
            TransferFunction transferFunction = transferFunctions[i][j];
            stringBuilder.append(transferFunction);
            if (j < numColumns - 1)
               stringBuilder.append(", ");
         }

         stringBuilder.append("\n");

      }


      return stringBuilder.toString();
   }


}
