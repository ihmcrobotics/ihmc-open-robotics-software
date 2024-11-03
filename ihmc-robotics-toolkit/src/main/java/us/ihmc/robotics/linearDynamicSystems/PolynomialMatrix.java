package us.ihmc.robotics.linearDynamicSystems;

import Jama.Matrix;
import us.ihmc.robotics.dataStructures.ObsoletePolynomial;

public class PolynomialMatrix
{
   private final ObsoletePolynomial[][] polynomials;

   public PolynomialMatrix(ObsoletePolynomial[][] polynomials)
   {
      this.polynomials = polynomials;
   }

   public static PolynomialMatrix constructSIMinusA(Matrix matrixA)
   {
      int order = matrixA.getRowDimension();
      if (matrixA.getColumnDimension() != order)
         throw new RuntimeException("Matrix A must be square!");

      ObsoletePolynomial[][] polynomials = new ObsoletePolynomial[order][order];

      for (int i = 0; i < order; i++)
      {
         for (int j = 0; j < order; j++)
         {
            if (i == j)
            {
               polynomials[i][j] = new ObsoletePolynomial(new double[] {1.0, -matrixA.get(i, j)});
            }
            else
            {
               polynomials[i][j] = new ObsoletePolynomial(new double[] {-matrixA.get(i, j)});
            }
         }
      }

      return new PolynomialMatrix(polynomials);
   }

   public ObsoletePolynomial getPolynomial(int row, int column)
   {
      return polynomials[row][column];
   }

   public ObsoletePolynomial computeDeterminant()
   {
      int order = polynomials.length;
      if (order == 1)
         return polynomials[0][0];

      ObsoletePolynomial ret = new ObsoletePolynomial(new double[] {0.0});

      for (int i = 0; i < order; i++)
      {
         ObsoletePolynomial multiplyingPolynomial = polynomials[0][i];
         ObsoletePolynomial cofactor = computeCofactor(0, i);

         ret = ret.plus(multiplyingPolynomial.times(cofactor));
      }

      return ret;
   }


   public ObsoletePolynomial[][] computeCofactors()
   {
      ObsoletePolynomial[][] ret = new ObsoletePolynomial[polynomials.length][polynomials[0].length];

      for (int i = 0; i < polynomials.length; i++)
      {
         for (int j = 0; j < polynomials[i].length; j++)
         {
            ret[i][j] = computeCofactor(i, j);
         }
      }

      return ret;
   }

   public ObsoletePolynomial computeCofactor(int row, int column)
   {
      int order = polynomials.length;
      if (order == 1)
         return new ObsoletePolynomial(new double[] {1.0});

      PolynomialMatrix rowAndColumnStriken = strikeRowAndColumn(row, column);
      ObsoletePolynomial Mij = rowAndColumnStriken.computeDeterminant();

      if (isEvenRowAndColumnSum(row, column))
      {
         return Mij;
      }

      return Mij.times(-1.0);
   }

   private boolean isEvenRowAndColumnSum(int row, int column)
   {
      return (((row + column) % 2) == 0);
   }

   private PolynomialMatrix strikeRowAndColumn(int rowToStrike, int columnToStrike)
   {
      int order = polynomials.length;
      if (order <= 1)
         throw new RuntimeException("Don't strike row and column if <= one!");

      int newOrder = order - 1;
      ObsoletePolynomial[][] newPolynomials = new ObsoletePolynomial[newOrder][newOrder];

      int row = 0;
      for (int i = 0; i < order; i++)
      {
         if (i == rowToStrike)
            continue;

         int column = 0;
         for (int j = 0; j < order; j++)
         {
            if (j == columnToStrike)
               continue;

            newPolynomials[row][column] = polynomials[i][j];
            column++;
         }

         row++;
      }

      return new PolynomialMatrix(newPolynomials);
   }
}
