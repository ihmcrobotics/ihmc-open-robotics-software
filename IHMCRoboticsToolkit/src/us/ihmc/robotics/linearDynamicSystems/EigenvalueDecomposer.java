package us.ihmc.robotics.linearDynamicSystems;

import java.util.ArrayList;

import org.ejml.data.CDenseMatrix64F;
import org.ejml.ops.CCommonOps;

import Jama.EigenvalueDecomposition;
import Jama.Matrix;
import us.ihmc.robotics.dataStructures.ComplexNumber;

public class EigenvalueDecomposer
{
   @SuppressWarnings("unused")
   private final Matrix matrixA;
   private final ComplexNumber[] eigenvalues;
   private final boolean[] isEigenvalueComplex;

   private final ComplexNumber[][] leftEigenvectors, rightEigenvectors;

   private final ArrayList<SingleRealMode> singleRealModes;
   private final ArrayList<ComplexConjugateMode> complexConjugateModes;


   public EigenvalueDecomposer(Matrix matrixA)
   {
      this.matrixA = matrixA;
      EigenvalueDecomposition eigenvalueDecomposition = new EigenvalueDecomposition(matrixA);

      isEigenvalueComplex = new boolean[matrixA.getRowDimension()];

      this.eigenvalues = extractEigenvaluesFromJamaMatrixD(eigenvalueDecomposition.getD(), isEigenvalueComplex);
      Matrix matrixV = eigenvalueDecomposition.getV();

      this.leftEigenvectors = extractLeftEigenvectorsFromJamaMatrixV(matrixV, isEigenvalueComplex);
      this.rightEigenvectors = extractRightEigenvectorsByInvertingComplexV(leftEigenvectors);

      this.singleRealModes = extractRealModes(eigenvalues, leftEigenvectors, rightEigenvectors, isEigenvalueComplex);
      this.complexConjugateModes = extractComplexConjugateModes(eigenvalues, leftEigenvectors, rightEigenvectors, isEigenvalueComplex);
   }

   private static ArrayList<ComplexConjugateMode> extractComplexConjugateModes(ComplexNumber[] eigenvalues, ComplexNumber[][] leftEigenvectors,
           ComplexNumber[][] rightEigenvectors, boolean[] isEigenvalueComplex)
   {
      int order = eigenvalues.length;
      int index = 0;

      ArrayList<ComplexConjugateMode> ret = new ArrayList<ComplexConjugateMode>();

      while (index < order)
      {
         if (isEigenvalueComplex[index])
         {
            ret.add(new ComplexConjugateMode(eigenvalues[index], leftEigenvectors[index], rightEigenvectors[index]));
            index++;
            index++;
         }
         else
         {
            index++;
         }
      }

      return ret;
   }

   private static ArrayList<SingleRealMode> extractRealModes(ComplexNumber[] eigenvalues, ComplexNumber[][] leftEigenvectors,
           ComplexNumber[][] rightEigenvectors, boolean[] isEigenvalueComplex)
   {
      int order = eigenvalues.length;
      int index = 0;

      ArrayList<SingleRealMode> ret = new ArrayList<SingleRealMode>();

      while (index < order)
      {
         if (isEigenvalueComplex[index])
         {
            index++;
         }
         else
         {
            ret.add(new SingleRealMode(eigenvalues[index], leftEigenvectors[index], rightEigenvectors[index]));
            index++;
         }
      }

      return ret;
   }

   private static ComplexNumber[][] extractRightEigenvectorsByInvertingComplexV(ComplexNumber[][] leftEigenvectors)
   {
      // How to get W??? matrixV.inverse() isn't the correct thing.
      // How about using A.transpose()? But then we have scaling and potential reordering problems!
      // Here we'll use a complex matrix inverse. But I have a hunch you could represent it as a
      // real Matrix and do an inverse on that and then extract the numbers since everything is in
      // complex conjugate pairs...

      int order = leftEigenvectors.length;
      
      // Load leftEigenvectors into new matrix as transpose
      ComplexMatrix ihmcComplexV = new ComplexMatrix(order, order);
      for (int i = 0; i < order; i++)
         for (int j = 0; j < order; j++)
            ihmcComplexV.set(i, j, leftEigenvectors[j][i]); // notice transpose here
      
      CDenseMatrix64F ejmlComplexV = ComplexTools.ihmcComplexToEjmlComplex(ihmcComplexV);

      if( !CCommonOps.invert(ejmlComplexV) ) {
         throw new RuntimeException("Complex matrix inversion failed");
      }
      
      return ComplexTools.copyEjmlComplexIntoIhmcComplexNumber2dArray(ejmlComplexV);
   }

   private static ComplexNumber[][] extractLeftEigenvectorsFromJamaMatrixV(Matrix matrixV, boolean[] isEigenvalueComplex)
   {
      int order = matrixV.getRowDimension();
      ComplexNumber[][] ret = new ComplexNumber[order][order];

      int index = 0;
      while (index < order)
      {
         if (isEigenvalueComplex[index])
         {
            ComplexNumber[][] twoComplexEigenvectors = extractTwoComplexLeftEigenvectors(matrixV, index, isEigenvalueComplex);
            ret[index] = twoComplexEigenvectors[0];
            index++;
            ret[index] = twoComplexEigenvectors[1];
            index++;
         }
         else
         {
            ComplexNumber[] realEigenvector = extractRealLeftEigenvector(matrixV, index);
            ret[index] = realEigenvector;
            index++;
         }
      }

      return ret;
   }

   private static ComplexNumber[] extractRealLeftEigenvector(Matrix matrix, int index)
   {
      int order = matrix.getRowDimension();

      ComplexNumber[] ret = new ComplexNumber[order];

      for (int i = 0; i < order; i++)
      {
         ret[i] = new ComplexNumber(matrix.get(i, index), 0.0);
      }

      return ret;
   }

   private static ComplexNumber[][] extractTwoComplexLeftEigenvectors(Matrix matrix, int column, boolean[] isEigenvalueComplex)
   {
      int order = matrix.getRowDimension();

      ComplexNumber[][] ret = new ComplexNumber[2][order];

      int index = 0;
      while (index < order)
      {
         ComplexNumber complexNumber = new ComplexNumber(matrix.get(index, column), matrix.get(index, column + 1));

         ret[0][index] = complexNumber;
         ret[1][index] = complexNumber.conj();

         index++;
      }

      return ret;
   }



   private static ComplexNumber[] extractEigenvaluesFromJamaMatrixD(Matrix matrixD, boolean[] packIsEigenvalueComplex)
   {
      int order = matrixD.getRowDimension();
      ComplexNumber[] ret = new ComplexNumber[order];

      int index = 0;
      while (index < order)
      {
         boolean isReal;
         if (index == order - 1)
         {
            // If only one left, it must be real.
            isReal = true;
         }
         else
         {
            // Otherwise, check to see if imaginary part is zero.
            // It would be nice to be able to check this a different way than this.
            // since this is error prone if the imaginary part is really small. We are
            // relying on JAMA to use 0.0 exactly.
            isReal = (matrixD.get(index, index + 1) == 0.0);
         }

         if (isReal)
         {
            packIsEigenvalueComplex[index] = false;
            ret[index] = new ComplexNumber(matrixD.get(index, index), 0.0);
            index++;
         }
         else
         {
            ComplexNumber[] twoComplexEigenvalues = extractTwoJamaRepresentedComplexNumbers(matrixD, index, index);
            packIsEigenvalueComplex[index] = true;
            ret[index] = twoComplexEigenvalues[0];
            index++;

            packIsEigenvalueComplex[index] = true;
            ret[index] = twoComplexEigenvalues[1];
            index++;
         }
      }

      return ret;
   }

   private static ComplexNumber[] extractTwoJamaRepresentedComplexNumbers(Matrix matrix, int i, int j)
   {
      double realOne = matrix.get(i, j);
      double imagOne = matrix.get(i, j + 1);
      double imagTwo = matrix.get(i + 1, j);
      double realTwo = matrix.get(i + 1, j + 1);

      ComplexNumber complexOne = new ComplexNumber(realOne, imagOne);
      ComplexNumber complexTwo = new ComplexNumber(realTwo, imagTwo);

      return new ComplexNumber[] {complexOne, complexTwo};
   }

   public ComplexNumber[] getEigenvalues()
   {
      return eigenvalues;
   }

   public ComplexNumber[][] getLeftEigenvectors()
   {
      return leftEigenvectors;
   }

   public ComplexNumber[][] getRightEigenvectors()
   {
      return rightEigenvectors;
   }

   public ArrayList<SingleRealMode> getRealModes()
   {
      return singleRealModes;
   }

   public ArrayList<ComplexConjugateMode> getComplexConjugateModes()
   {
      return complexConjugateModes;
   }

}
