package us.ihmc.robotics.linearDynamicSystems;

import org.ejml.simple.SimpleMatrix;
import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialBasics;

public class PolynomialMatrix
{
   private final PolynomialBasics[][] polynomials;

   public PolynomialMatrix(PolynomialBasics[][] polynomials)
   {
      this.polynomials = polynomials;
   }

   public static PolynomialMatrix constructSIMinusA(SimpleMatrix matrixA)
   {
      int order = matrixA.numRows();
      if (matrixA.numCols() != order)
         throw new RuntimeException("Matrix A must be square!");

      Polynomial[][] polynomials = new Polynomial[order][order];

      for (int i = 0; i < order; i++)
      {
         for (int j = 0; j < order; j++)
         {
            if (i == j)
            {
               polynomials[i][j] = new Polynomial(-matrixA.get(i, j), 1.0);
            }
            else
            {
               polynomials[i][j] = new Polynomial(-matrixA.get(i, j));
            }
         }
      }

      return new PolynomialMatrix(polynomials);
   }

   public PolynomialBasics getPolynomial(int row, int column)
   {
      return polynomials[row][column];
   }

   public PolynomialBasics computeDeterminant()
   {
      int order = polynomials.length;
      if (order == 1)
         return polynomials[0][0];

      PolynomialBasics ret = new Polynomial(0.0);

      for (int i = 0; i < order; i++)
      {
         PolynomialBasics multiplyingPolynomial = polynomials[0][i];
         PolynomialBasics cofactor = computeCofactor(0, i);

         ret = ret.plus(multiplyingPolynomial.times(cofactor));
      }

      return ret;
   }


   public PolynomialBasics[][] computeCofactors()
   {
      PolynomialBasics[][] ret = new Polynomial[polynomials.length][polynomials[0].length];

      for (int i = 0; i < polynomials.length; i++)
      {
         for (int j = 0; j < polynomials[i].length; j++)
         {
            ret[i][j] = computeCofactor(i, j);
         }
      }

      return ret;
   }

   public PolynomialBasics computeCofactor(int row, int column)
   {
      int order = polynomials.length;
      if (order == 1)
         return new Polynomial(new double[] {1.0});

      PolynomialMatrix rowAndColumnStriken = strikeRowAndColumn(row, column);
      PolynomialBasics Mij = rowAndColumnStriken.computeDeterminant();

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
      PolynomialBasics[][] newPolynomials = new Polynomial[newOrder][newOrder];

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
