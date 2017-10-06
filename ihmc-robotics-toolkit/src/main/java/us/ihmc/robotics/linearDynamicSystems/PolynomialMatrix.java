package us.ihmc.robotics.linearDynamicSystems;

import Jama.Matrix;
import us.ihmc.robotics.dataStructures.Polynomial;

public class PolynomialMatrix
{
   private final Polynomial[][] polynomials;

   public PolynomialMatrix(Polynomial[][] polynomials)
   {
      this.polynomials = polynomials;
   }

   public static PolynomialMatrix constructSIMinusA(Matrix matrixA)
   {
      int order = matrixA.getRowDimension();
      if (matrixA.getColumnDimension() != order)
         throw new RuntimeException("Matrix A must be square!");

      Polynomial[][] polynomials = new Polynomial[order][order];

      for (int i = 0; i < order; i++)
      {
         for (int j = 0; j < order; j++)
         {
            if (i == j)
            {
               polynomials[i][j] = new Polynomial(new double[] {1.0, -matrixA.get(i, j)});
            }
            else
            {
               polynomials[i][j] = new Polynomial(new double[] {-matrixA.get(i, j)});
            }
         }
      }

      return new PolynomialMatrix(polynomials);
   }

   public Polynomial getPolynomial(int row, int column)
   {
      return polynomials[row][column];
   }

   public Polynomial computeDeterminant()
   {
      int order = polynomials.length;
      if (order == 1)
         return polynomials[0][0];

      Polynomial ret = new Polynomial(new double[] {0.0});

      for (int i = 0; i < order; i++)
      {
         Polynomial multiplyingPolynomial = polynomials[0][i];
         Polynomial cofactor = computeCofactor(0, i);

         ret = ret.plus(multiplyingPolynomial.times(cofactor));
      }

      return ret;
   }


   public Polynomial[][] computeCofactors()
   {
      Polynomial[][] ret = new Polynomial[polynomials.length][polynomials[0].length];

      for (int i = 0; i < polynomials.length; i++)
      {
         for (int j = 0; j < polynomials[i].length; j++)
         {
            ret[i][j] = computeCofactor(i, j);
         }
      }

      return ret;
   }

   public Polynomial computeCofactor(int row, int column)
   {
      int order = polynomials.length;
      if (order == 1)
         return new Polynomial(new double[] {1.0});

      PolynomialMatrix rowAndColumnStriken = strikeRowAndColumn(row, column);
      Polynomial Mij = rowAndColumnStriken.computeDeterminant();

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
      Polynomial[][] newPolynomials = new Polynomial[newOrder][newOrder];

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
